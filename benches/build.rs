use std::{io::Read, time::Duration};

use bvh::bvh::BVH;
use criterion::{criterion_group, criterion_main, BatchSize, BenchmarkId, Criterion};
use mint::Point3;
use obj::{IndexTuple, ObjData};
use sbvh::Sbvh;

#[derive(Clone, Debug)]
struct Polygon {
    points: Vec<Point3<f32>>,

    sbvh_bounding_box: sbvh::Aabb,

    bvh_bounding_box: bvh::aabb::AABB,
    bvh_node_idx: usize,
}

impl sbvh::Primitive for Polygon {
    fn points(&self) -> &[Point3<f32>] {
        &self.points
    }

    fn bounding_box(&self) -> &sbvh::Aabb {
        &self.sbvh_bounding_box
    }

    fn split(&self, _split: sbvh::Split) -> (Self, Option<Self>) {
        unimplemented!();
    }
}

impl bvh::aabb::Bounded for Polygon {
    fn aabb(&self) -> bvh::aabb::AABB {
        self.bvh_bounding_box
    }
}

impl bvh::bounding_hierarchy::BHShape for Polygon {
    fn set_bh_node_index(&mut self, i: usize) {
        self.bvh_node_idx = i;
    }

    fn bh_node_index(&self) -> usize {
        self.bvh_node_idx
    }
}

fn load_obj<R: Read>(data: R) -> Vec<Polygon> {
    let obj_data = ObjData::load_buf(data).unwrap();
    obj_data
        .objects
        .iter()
        .flat_map(|object| {
            object.groups.iter().flat_map(|group| {
                group.polys.iter().map(|poly| {
                    let mut points = Vec::new();

                    for IndexTuple(ipos, _, _) in &poly.0 {
                        points.push(Point3 {
                            x: obj_data.position[*ipos][0],
                            y: obj_data.position[*ipos][1],
                            z: obj_data.position[*ipos][2],
                        });
                    }

                    let sbvh_bounding_box = sbvh::Aabb::from_points(&points);

                    let bvh_bounding_box = bvh::aabb::AABB::with_bounds(
                        bvh::Vector3::new(
                            sbvh_bounding_box.min.x,
                            sbvh_bounding_box.min.y,
                            sbvh_bounding_box.min.z,
                        ),
                        bvh::Vector3::new(
                            sbvh_bounding_box.max.x,
                            sbvh_bounding_box.max.y,
                            sbvh_bounding_box.max.z,
                        ),
                    );

                    Polygon {
                        points,
                        sbvh_bounding_box,
                        bvh_bounding_box,
                        bvh_node_idx: 0,
                    }
                })
            })
        })
        .collect()
}

fn build(c: &mut Criterion) {
    let suzanne = load_obj(&include_bytes!("./suzanne.obj")[..]);
    let subdivided_suzanne = load_obj(&include_bytes!("./subdivided_suzanne.obj")[..]);

    let mut group = c.benchmark_group("build");
    for model in [suzanne, subdivided_suzanne].iter() {
        group.bench_with_input(BenchmarkId::new("sbvh", model.len()), model, |b, model| {
            b.iter(|| Sbvh::new(model));
        });
        group.bench_with_input(BenchmarkId::new("bvh", model.len()), model, |b, model| {
            b.iter_batched(
                move || model.clone(),
                |mut model| BVH::build(&mut model),
                BatchSize::LargeInput,
            );
        });
    }

    group.finish();
}

criterion_group! {
    name = benches;
    config = Criterion::default().sample_size(50).measurement_time(Duration::from_secs(10));
    targets = build
}

criterion_main!(benches);
