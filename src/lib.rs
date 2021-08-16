#![feature(new_uninit)]

use std::{
    cell::UnsafeCell,
    fmt::{self, Debug},
    marker::PhantomData,
    mem::MaybeUninit,
    sync::atomic::{AtomicUsize, Ordering},
};

use mint::Point3;
use num_traits::{Float, NumCast};

// TODO Un-hardcode this someday
const OBJECT_BUCKETS: usize = 32;

#[derive(Debug)]
struct AtomicArena<T> {
    storage: UnsafeCell<Box<[MaybeUninit<T>]>>,
    len: AtomicUsize,
}

unsafe impl<T: Send> Send for AtomicArena<T> {}
unsafe impl<T: Sync> Sync for AtomicArena<T> {}

impl<T> AtomicArena<T> {
    fn new(size: usize) -> Self {
        Self {
            storage: UnsafeCell::new(Box::new_uninit_slice(size)),
            len: AtomicUsize::new(0),
        }
    }

    fn push(&self, v: T) -> usize {
        let idx = self.len.fetch_add(1, Ordering::SeqCst);
        unsafe {
            (&mut *self.storage.get())[idx] = MaybeUninit::new(v);
        }
        idx
    }

    fn get(&self, i: usize) -> Option<&T> {
        if i < self.len.load(Ordering::SeqCst) {
            unsafe { Some((&*self.storage.get())[i].assume_init_ref()) }
        } else {
            None
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Axis {
    X,
    Y,
    Z,
}

trait GetAxis {
    type Num: Float + Debug + Send + Sync;
    fn axis(&self, axis: Axis) -> Self::Num;
}

impl<N: Float + Debug + Send + Sync> GetAxis for Point3<N> {
    type Num = N;
    fn axis(&self, axis: Axis) -> N {
        match axis {
            Axis::X => self.x,
            Axis::Y => self.y,
            Axis::Z => self.z,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Split<N: Float + Debug + Send + Sync> {
    pub axis: Axis,
    pub position: N,
}

#[derive(Clone, Debug)]
pub struct Aabb<N: Float + Debug + Send + Sync> {
    min: Point3<N>,
    max: Point3<N>,
}

impl<N: Float + Debug + Send + Sync> Aabb<N> {
    pub fn new(min: Point3<N>, max: Point3<N>) -> Self {
        Self { min, max }
    }

    pub fn from_points(points: &[Point3<N>]) -> Self {
        let mut min = Point3 {
            x: N::max_value(),
            y: N::max_value(),
            z: N::max_value(),
        };
        let mut max = Point3 {
            x: N::min_value(),
            y: N::min_value(),
            z: N::min_value(),
        };

        for point in points {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);

            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
        }

        Self::new(min, max)
    }

    pub fn centroid(&self) -> Point3<N> {
        let two: N = NumCast::from(2).unwrap();

        Point3 {
            x: (self.min.x + self.max.x) / two,
            y: (self.min.y + self.max.y) / two,
            z: (self.min.z + self.max.z) / two,
        }
    }

    pub fn union(&self, other: &Self) -> Self {
        let min = Point3 {
            x: self.min.x.min(other.min.x),
            y: self.min.y.min(other.min.y),
            z: self.min.z.min(other.min.z),
        };
        let max = Point3 {
            x: self.max.x.max(other.max.x),
            y: self.max.y.max(other.max.y),
            z: self.max.z.max(other.max.z),
        };

        Self::new(min, max)
    }

    pub fn surface_area(&self) -> N {
        let xs = self.max.x - self.min.x;
        let ys = self.max.y - self.min.y;
        let zs = self.max.z - self.min.z;

        let two: N = NumCast::from(2).unwrap();

        two * xs * ys + two * xs * zs + two * ys * zs
    }

    pub fn extent(&self, axis: Axis) -> N {
        self.max.axis(axis) - self.min.axis(axis)
    }
}

impl<N: Float + Debug + Send + Sync> Default for Aabb<N> {
    fn default() -> Self {
        Self::new(
            Point3 {
                x: N::max_value(),
                y: N::max_value(),
                z: N::max_value(),
            },
            Point3 {
                x: N::min_value(),
                y: N::min_value(),
                z: N::min_value(),
            },
        )
    }
}

pub trait Primitive: Sized + Send + Sync {
    type Num: Float + Debug + Send + Sync;

    fn points(&self) -> &[Point3<Self::Num>];
    fn split(&self, split: Split<Self::Num>) -> (Self, Option<Self>);

    fn bounding_box(&self) -> &Aabb<Self::Num>;
}

pub struct Reference<P: Primitive> {
    pub primitive_idx: usize,
    pub bounding_box: Aabb<P::Num>,

    _primitive_ty: PhantomData<P>,
}

impl<P: Primitive> Clone for Reference<P> {
    fn clone(&self) -> Self {
        Self {
            primitive_idx: self.primitive_idx,
            bounding_box: self.bounding_box.clone(),

            _primitive_ty: PhantomData,
        }
    }
}

impl<P: Primitive> Debug for Reference<P> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("Reference")
            .field("primitive_idx", &self.primitive_idx)
            .field("bounding_box", &self.bounding_box)
            .finish()
    }
}

#[derive(Clone, Debug)]
struct ObjectBucket<N: Float + Debug + Send + Sync> {
    bounding_box: Aabb<N>,
    count: usize,
}

impl<N: Float + Debug + Send + Sync> Default for ObjectBucket<N> {
    fn default() -> Self {
        Self {
            bounding_box: Aabb::default(),
            count: 0,
        }
    }
}

// TODO Tighten total bounding box to only cover centroids
fn object_split_candidate<P: Primitive>(
    references: &[Reference<P>],
    aabb_total: &Aabb<P::Num>,
    input: &[usize],
    split_axis: Axis,
) -> (P::Num, usize) {
    let mut buckets: [ObjectBucket<P::Num>; OBJECT_BUCKETS] = Default::default();
    let extent = aabb_total.extent(split_axis);

    // Place references into buckets
    for idx in input {
        let bucket_idx: usize = NumCast::from(
            ((references[*idx].bounding_box.centroid().axis(split_axis)
                - aabb_total.min.axis(split_axis))
                / extent
                * NumCast::from(OBJECT_BUCKETS).unwrap())
            .floor(),
        )
        .unwrap();

        buckets[bucket_idx].count += 1;
        buckets[bucket_idx].bounding_box = buckets[bucket_idx]
            .bounding_box
            .union(&references[*idx].bounding_box);
    }

    // Compute all split left-hand sides
    let mut lhs_splits: [(usize, Aabb<P::Num>); OBJECT_BUCKETS - 1] = Default::default();
    let mut lhs_split = (0usize, Aabb::default());
    for i in 0..OBJECT_BUCKETS - 1 {
        lhs_split.0 += buckets[i].count;
        lhs_split.1 = lhs_split.1.union(&buckets[i].bounding_box);
        lhs_splits[i] = lhs_split.clone();
    }

    // Compute split rhs and find split with minimum SAH
    let mut rhs_split = (0usize, Aabb::default());
    let mut min_cost = P::Num::max_value();
    let mut min_idx = 0usize;
    for i in (1..OBJECT_BUCKETS).rev() {
        rhs_split.0 += buckets[i].count;
        rhs_split.1 = rhs_split.1.union(&buckets[i].bounding_box);

        let lhs_split = &lhs_splits[i - 1];

        let traverse_cost: P::Num = NumCast::from(1).unwrap();
        let n_lhs: P::Num = NumCast::from(lhs_split.0).unwrap();
        let n_rhs: P::Num = NumCast::from(rhs_split.0).unwrap();
        let cost = traverse_cost
            + (n_lhs * lhs_split.1.surface_area() + n_rhs * rhs_split.1.surface_area())
                / aabb_total.surface_area();

        if cost < min_cost {
            min_cost = cost;
            min_idx = i;
        }
    }

    (min_cost, min_idx)
}

fn object_split<P: Primitive>(
    references: &[Reference<P>],
    aabb_total: &Aabb<P::Num>,
    input: &[usize],
    output: &mut [usize],
    split_axis: Axis,
    bucket_idx: usize,
) -> usize {
    let extent = aabb_total.extent(split_axis);

    let mut lhs_ptr = 0usize;
    let mut rhs_ptr = output.len() - 1;
    for idx in input {
        let this_bucket: usize = NumCast::from(
            ((references[*idx].bounding_box.centroid().axis(split_axis)
                - aabb_total.min.axis(split_axis))
                / extent
                * NumCast::from(OBJECT_BUCKETS).unwrap())
            .floor(),
        )
        .unwrap();

        if this_bucket < bucket_idx {
            output[lhs_ptr] = *idx;
            lhs_ptr += 1;
        } else {
            output[rhs_ptr] = *idx;
            rhs_ptr -= 1;
        }
    }

    lhs_ptr
}

#[derive(Debug)]
pub enum SbvhNode<P: Primitive> {
    Node { lhs: usize, rhs: usize },
    Leaf { references: Vec<Reference<P>> },
}

#[derive(Debug)]
pub struct Sbvh<P: Primitive> {
    nodes: AtomicArena<SbvhNode<P>>,
    root_node: usize,
}

impl<P: Primitive> Sbvh<P> {
    pub fn new(primitives: &[P]) -> Self {
        // Construct initial references
        let references = primitives
            .iter()
            .enumerate()
            .map(|(i, primitive)| Reference {
                primitive_idx: i,
                bounding_box: primitive.bounding_box().clone(),
                _primitive_ty: PhantomData,
            })
            .collect::<Vec<_>>();

        // Create double buffers for partitioning
        let mut buffer_a = Vec::with_capacity(references.len());
        for i in 0..references.len() {
            buffer_a.push(i);
        }
        let mut buffer_b = vec![0usize; references.len()];

        // Create output arena
        let output_arena = AtomicArena::new(2 * references.len() - 1);

        // Perform recursive build
        let root_node = Self::build(&references, &mut buffer_a, &mut buffer_b, &output_arena);

        Self {
            nodes: output_arena,
            root_node,
        }
    }

    fn build(
        references: &[Reference<P>],
        input_buffer: &mut [usize],
        output_buffer: &mut [usize],
        output_arena: &AtomicArena<SbvhNode<P>>,
    ) -> usize {
        // Just return a leaf if we only have one reference
        if input_buffer.len() < 2 {
            return Self::create_leaf(references, input_buffer, output_arena);
        }

        // Compute total AABB of all references
        let mut aabb_total = Aabb::default();
        for idx in input_buffer.iter() {
            aabb_total = aabb_total.union(&references[*idx].bounding_box);
        }

        // Compute split axis based on maximum extent
        let split_axis = if aabb_total.extent(Axis::X) >= aabb_total.extent(Axis::Y)
            && aabb_total.extent(Axis::X) >= aabb_total.extent(Axis::Z)
        {
            Axis::X
        } else if aabb_total.extent(Axis::Y) >= aabb_total.extent(Axis::Z) {
            Axis::Y
        } else {
            Axis::Z
        };

        // Compute leaf cost
        let leaf_cost: P::Num = NumCast::from(input_buffer.len()).unwrap();

        // Try object split
        let (object_split_cost, object_split_bucket) =
            object_split_candidate(references, &aabb_total, input_buffer, split_axis);

        if object_split_cost < leaf_cost {
            let split_point = object_split(
                references,
                &aabb_total,
                input_buffer,
                output_buffer,
                split_axis,
                object_split_bucket,
            );

            let (a_in, b_in) = output_buffer.split_at_mut(split_point);
            let (a_out, b_out) = input_buffer.split_at_mut(split_point);

            let (lhs, rhs) = rayon::join(
                || Self::build(references, a_in, a_out, output_arena),
                || Self::build(references, b_in, b_out, output_arena),
            );

            output_arena.push(SbvhNode::Node { lhs, rhs })
        } else {
            Self::create_leaf(references, input_buffer, output_arena)
        }
    }

    fn create_leaf(
        references: &[Reference<P>],
        indices: &[usize],
        output_arena: &AtomicArena<SbvhNode<P>>,
    ) -> usize {
        output_arena.push(SbvhNode::Leaf {
            references: indices.iter().map(|idx| references[*idx].clone()).collect(),
        })
    }

    fn node_bounding_box(&self, idx: usize) -> Aabb<P::Num> {
        let node = self.nodes.get(idx).unwrap();
        match node {
            SbvhNode::Node { lhs, rhs } => self
                .node_bounding_box(*lhs)
                .union(&self.node_bounding_box(*rhs)),
            SbvhNode::Leaf { references } => {
                let mut aabb = Aabb::default();
                for reference in references {
                    aabb = aabb.union(&reference.bounding_box)
                }
                aabb
            }
        }
    }

    pub fn bounding_box(&self) -> Aabb<P::Num> {
        self.node_bounding_box(self.root_node)
    }
}
