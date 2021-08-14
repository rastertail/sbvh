use std::marker::PhantomData;

use mint::{Point3, Vector3};
use num_traits::Float;

pub enum Axis {
    X,
    Y,
    Z,
}

pub struct Split<N: Float> {
    pub axis: Axis,
    pub position: N,
}

pub struct Aabb<N: Float> {
    min: Point3<N>,
    max: Point3<N>,
}

impl<N: Float> Aabb<N> {
    pub fn new(min: Point3<N>, max: Point3<N>) -> Self {
        Self { min, max }
    }

    pub fn tighten(&mut self, points: &[Point3<N>]) {
        unimplemented!();
    }

    pub fn split(&self, split: Split<N>) -> (Self, Self) {
        unimplemented!();
    }
}

pub trait Primitive {
    type Num: Float;

    fn points(&self) -> &[Point3<Self::Num>];
    fn bounding_box(&self) -> Aabb<Self::Num>;
}

pub struct Reference<P: Primitive> {
    primitive_idx: usize,
    bounding_box: Aabb<P::Num>,

    _primitive_ty: PhantomData<P>,
}

pub enum Sbvh<P: Primitive> {
    Node {
        aabb: Aabb<P::Num>,
        lhs: P,
        rhs: P,
    },
    Leaf {
        aabb: Aabb<P::Num>,
        references: Vec<Reference<P>>, // TODO optimize with tinyvec
    },
}
