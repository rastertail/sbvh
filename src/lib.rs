#![feature(new_uninit)]

use std::{
    cell::UnsafeCell,
    fmt::{self, Debug},
    mem::MaybeUninit,
    ptr,
    sync::atomic::{AtomicUsize, Ordering},
};

use mint::Point3;

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

// TODO Figure out how to drop everything at once
impl<T> Drop for AtomicArena<T> {
    fn drop(&mut self) {
        for i in 0..self.len.load(Ordering::SeqCst) {
            unsafe { ptr::drop_in_place(self.storage.get_mut()[i].as_mut_ptr()) }
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
    fn axis(&self, axis: Axis) -> f32;
}

impl GetAxis for Point3<f32> {
    fn axis(&self, axis: Axis) -> f32 {
        match axis {
            Axis::X => self.x,
            Axis::Y => self.y,
            Axis::Z => self.z,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Split {
    pub axis: Axis,
    pub position: f32,
}

#[derive(Clone, Debug)]
pub struct Aabb {
    pub min: Point3<f32>,
    pub max: Point3<f32>,
}

impl Aabb {
    pub fn new(min: Point3<f32>, max: Point3<f32>) -> Self {
        Self { min, max }
    }

    pub fn from_points(points: &[Point3<f32>]) -> Self {
        let mut min = Point3 {
            x: f32::MAX,
            y: f32::MAX,
            z: f32::MAX,
        };
        let mut max = Point3 {
            x: f32::MIN,
            y: f32::MIN,
            z: f32::MIN,
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

    pub fn centroid(&self) -> Point3<f32> {
        Point3 {
            x: (self.min.x + self.max.x) / 2.0,
            y: (self.min.y + self.max.y) / 2.0,
            z: (self.min.z + self.max.z) / 2.0,
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

    pub fn surface_area(&self) -> f32 {
        let xs = self.max.x - self.min.x;
        let ys = self.max.y - self.min.y;
        let zs = self.max.z - self.min.z;

        2.0 * xs * ys + 2.0 * xs * zs + 2.0 * ys * zs
    }

    pub fn extent(&self, axis: Axis) -> f32 {
        self.max.axis(axis) - self.min.axis(axis)
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self::new(
            Point3 {
                x: f32::MAX,
                y: f32::MAX,
                z: f32::MAX,
            },
            Point3 {
                x: f32::MIN,
                y: f32::MIN,
                z: f32::MIN,
            },
        )
    }
}

pub trait Primitive: Sized + Send + Sync {
    fn points(&self) -> &[Point3<f32>];
    fn split(&self, split: Split) -> (Self, Option<Self>);

    fn bounding_box(&self) -> &Aabb;
}

pub struct Reference {
    pub primitive_idx: usize,
    pub bounding_box: Aabb,
}

impl Clone for Reference {
    fn clone(&self) -> Self {
        Self {
            primitive_idx: self.primitive_idx,
            bounding_box: self.bounding_box.clone(),
        }
    }
}

impl Debug for Reference {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("Reference")
            .field("primitive_idx", &self.primitive_idx)
            .field("bounding_box", &self.bounding_box)
            .finish()
    }
}

#[derive(Clone, Debug)]
struct ObjectBucket {
    bounding_box: Aabb,
    count: usize,
}

impl Default for ObjectBucket {
    fn default() -> Self {
        Self {
            bounding_box: Aabb::default(),
            count: 0,
        }
    }
}

// TODO Tighten total bounding box to only cover centroids
fn object_split_candidate(
    references: &[Reference],
    aabb_total: &Aabb,
    input: &[usize],
    split_axis: Axis,
) -> (f32, usize) {
    let mut buckets: [ObjectBucket; OBJECT_BUCKETS] = Default::default();
    let extent = aabb_total.extent(split_axis);

    // Place references into buckets
    for idx in input {
        let bucket_idx = ((references[*idx].bounding_box.centroid().axis(split_axis)
            - aabb_total.min.axis(split_axis))
            / extent
            * (OBJECT_BUCKETS as f32)) as usize;

        buckets[bucket_idx].count += 1;
        buckets[bucket_idx].bounding_box = buckets[bucket_idx]
            .bounding_box
            .union(&references[*idx].bounding_box);
    }

    // Compute all split left-hand sides
    let mut lhs_splits: [(usize, Aabb); OBJECT_BUCKETS - 1] = Default::default();
    let mut lhs_split = (0usize, Aabb::default());
    for i in 0..OBJECT_BUCKETS - 1 {
        lhs_split.0 += buckets[i].count;
        lhs_split.1 = lhs_split.1.union(&buckets[i].bounding_box);
        lhs_splits[i] = lhs_split.clone();
    }

    // Compute split rhs and find split with minimum SAH
    let mut rhs_split = (0usize, Aabb::default());
    let mut min_cost = f32::MAX;
    let mut min_idx = 0usize;
    for i in (1..OBJECT_BUCKETS).rev() {
        rhs_split.0 += buckets[i].count;
        rhs_split.1 = rhs_split.1.union(&buckets[i].bounding_box);

        let lhs_split = &lhs_splits[i - 1];

        let traverse_cost = 1.0f32;
        let n_lhs = lhs_split.0 as f32;
        let n_rhs = rhs_split.0 as f32;
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

fn object_split(
    references: &[Reference],
    aabb_total: &Aabb,
    input: &[usize],
    output: &mut [usize],
    split_axis: Axis,
    bucket_idx: usize,
) -> usize {
    let extent = aabb_total.extent(split_axis);

    let mut lhs_ptr = 0usize;
    let mut rhs_ptr = output.len() - 1;
    for idx in input {
        let this_bucket = ((references[*idx].bounding_box.centroid().axis(split_axis)
            - aabb_total.min.axis(split_axis))
            / extent
            * (OBJECT_BUCKETS as f32)) as usize;

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
pub enum SbvhNode {
    Node { lhs: usize, rhs: usize },
    Leaf { references: Vec<Reference> },
}

#[derive(Debug)]
pub struct Sbvh {
    nodes: AtomicArena<SbvhNode>,
    root_node: usize,
}

impl Sbvh {
    pub fn new<P: Primitive>(primitives: &[P]) -> Self {
        // Construct initial references
        let references = primitives
            .iter()
            .enumerate()
            .map(|(i, primitive)| Reference {
                primitive_idx: i,
                bounding_box: primitive.bounding_box().clone(),
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
        references: &[Reference],
        input_buffer: &mut [usize],
        output_buffer: &mut [usize],
        output_arena: &AtomicArena<SbvhNode>,
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
        let leaf_cost = input_buffer.len() as f32;

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
        references: &[Reference],
        indices: &[usize],
        output_arena: &AtomicArena<SbvhNode>,
    ) -> usize {
        output_arena.push(SbvhNode::Leaf {
            references: indices.iter().map(|idx| references[*idx].clone()).collect(),
        })
    }

    fn node_bounding_box(&self, idx: usize) -> Aabb {
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

    pub fn bounding_box(&self) -> Aabb {
        self.node_bounding_box(self.root_node)
    }
}
