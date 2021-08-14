use std::{
    cmp::Ordering,
    fmt::{self, Debug},
    marker::PhantomData,
};

use mint::Point3;
use num_traits::{Float, NumCast, Zero};

#[derive(Clone, Debug)]
pub enum Axis {
    X,
    Y,
    Z,
}

#[derive(Clone, Debug)]
pub struct Split<N: Float + Debug> {
    pub axis: Axis,
    pub position: N,
}

#[derive(Clone, Debug)]
pub struct Aabb<N: Float + Debug> {
    min: Point3<N>,
    max: Point3<N>,
}

impl<N: Float + Debug> Aabb<N> {
    pub fn new(min: Point3<N>, max: Point3<N>) -> Self {
        debug_assert!(min.x <= max.x);
        debug_assert!(min.y <= max.y);
        debug_assert!(min.z <= max.z);

        Self { min, max }
    }

    pub fn zero() -> Self {
        let zero: N = Zero::zero();

        let min = Point3 {
            x: zero,
            y: zero,
            z: zero,
        };
        let max = Point3 {
            x: zero,
            y: zero,
            z: zero,
        };

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
}

pub trait Primitive: Sized {
    type Num: Float + Debug;

    fn points(&self) -> &[Point3<Self::Num>];
    fn split(&self, split: Split<Self::Num>) -> (Self, Option<Self>);

    fn bounding_box(&self) -> Aabb<Self::Num> {
        let zero: Self::Num = Zero::zero();

        let mut min = Point3 {
            x: zero,
            y: zero,
            z: zero,
        };
        let mut max = Point3 {
            x: zero,
            y: zero,
            z: zero,
        };

        for point in self.points() {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);

            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
        }

        Aabb::new(min, max)
    }
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

fn surface_area_heuristic<P: Primitive>(
    a: &[Reference<P>],
    b: &[Reference<P>],
    aabb_total: &Aabb<P::Num>,
) -> P::Num {
    let mut aabb_a = Aabb::zero();
    let mut aabb_b = Aabb::zero();

    for reference in a {
        aabb_a = aabb_a.union(&reference.bounding_box);
    }

    for reference in b {
        aabb_b = aabb_b.union(&reference.bounding_box);
    }

    let traverse_cost: P::Num = NumCast::from(0.175).unwrap();
    let n_a: P::Num = NumCast::from(a.len()).unwrap();
    let n_b: P::Num = NumCast::from(b.len()).unwrap();

    traverse_cost
        + (aabb_a.surface_area() / aabb_total.surface_area()) * n_a
        + (aabb_b.surface_area() / aabb_total.surface_area()) * n_b
}

#[derive(Clone, Debug)]
pub enum Sbvh<P: Primitive> {
    Node {
        lhs: Box<Self>,
        rhs: Box<Self>,
    },
    Leaf {
        references: Vec<Reference<P>>, // TODO optimize with tinyvec
    },
}

impl<P: Primitive> Sbvh<P> {
    pub fn new(primitives: &[P]) -> Box<Self> {
        // Construct initial references
        let references = primitives
            .iter()
            .enumerate()
            .map(|(i, primitive)| Reference {
                primitive_idx: i,
                bounding_box: primitive.bounding_box(),
                _primitive_ty: PhantomData,
            })
            .collect::<Vec<_>>();

        // Perform recursive build
        Self::build(&references)
    }

    fn build(references: &[Reference<P>]) -> Box<Self> {
        // Compute total AABB of all references
        let mut aabb_total = Aabb::zero();
        for reference in references {
            aabb_total = aabb_total.union(&reference.bounding_box);
        }

        // Sort references by their AABB centroids on each axis
        let mut x_sorted = references.to_vec();
        let mut y_sorted = references.to_vec();
        let mut z_sorted = references.to_vec();

        x_sorted.sort_by(|ref_a, ref_b| {
            ref_a
                .bounding_box
                .centroid()
                .x
                .partial_cmp(&ref_b.bounding_box.centroid().x)
                .unwrap_or(Ordering::Equal)
        });

        y_sorted.sort_by(|ref_a, ref_b| {
            ref_a
                .bounding_box
                .centroid()
                .y
                .partial_cmp(&ref_b.bounding_box.centroid().y)
                .unwrap_or(Ordering::Equal)
        });

        z_sorted.sort_by(|ref_a, ref_b| {
            ref_a
                .bounding_box
                .centroid()
                .z
                .partial_cmp(&ref_b.bounding_box.centroid().z)
                .unwrap_or(Ordering::Equal)
        });

        // Pick a candidate for an object split if possible
        if references.len() >= 2 {
            let (os_sah, os_a, os_b) = (1..references.len())
                .map(|i| {
                    let (xa, xb) = x_sorted.split_at(i);
                    let (ya, yb) = y_sorted.split_at(i);
                    let (za, zb) = z_sorted.split_at(i);

                    let x_sah = surface_area_heuristic(xa, xb, &aabb_total);
                    let y_sah = surface_area_heuristic(ya, yb, &aabb_total);
                    let z_sah = surface_area_heuristic(za, zb, &aabb_total);

                    if x_sah <= y_sah && x_sah <= z_sah {
                        (x_sah, xa, xb)
                    } else if y_sah <= x_sah && y_sah <= z_sah {
                        (y_sah, ya, yb)
                    } else {
                        (z_sah, za, zb)
                    }
                })
                .min_by(|(sah_a, _, _), (sah_b, _, _)| {
                    sah_a.partial_cmp(sah_b).unwrap_or(Ordering::Equal)
                })
                .unwrap();

            // TODO Create spatial split candidate

            // Either use object split or just create a leaf
            Box::new(if os_sah < NumCast::from(references.len()).unwrap() {
                Self::Node {
                    lhs: Self::build(os_a),
                    rhs: Self::build(os_b),
                }
            } else {
                Self::Leaf {
                    references: references.to_vec(),
                }
            })
        } else {
            // Create leaf if no object split was possible
            Box::new(Self::Leaf {
                references: references.to_vec(),
            })
        }
    }

    pub fn bounding_box(&self) -> Aabb<P::Num> {
        match self {
            Sbvh::Node { lhs, rhs } => lhs.bounding_box().union(&rhs.bounding_box()),
            Sbvh::Leaf { references } => {
                let mut aabb = Aabb::zero();
                for reference in references {
                    aabb = aabb.union(&reference.bounding_box)
                }
                aabb
            }
        }
    }
}
