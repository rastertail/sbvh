use std::time::Instant;

use mint::Point3;
use obj::{IndexTuple, ObjData};
use sbvh::{Aabb, Primitive, Sbvh, Split};

#[derive(Debug)]
struct Polygon {
    points: Vec<Point3<f32>>,

    bounding_box: Aabb<f32>,
}

impl Primitive for Polygon {
    type Num = f32;

    fn points(&self) -> &[Point3<f32>] {
        &self.points
    }

    fn bounding_box(&self) -> Aabb<f32> {
        self.bounding_box.clone()
    }

    fn split(&self, _split: Split<f32>) -> (Self, Option<Self>) {
        unimplemented!();
    }
}

fn main() {
    let start = Instant::now();
    let obj_data = ObjData::load_buf(&include_bytes!("./subdivided_suzanne.obj")[..]).unwrap();
    let polygons = obj_data
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

                    let bounding_box = Aabb::from_points(&points);

                    Polygon {
                        points,
                        bounding_box,
                    }
                })
            })
        })
        .collect::<Vec<_>>();
    println!("Loaded {} polys in {:?}", polygons.len(), start.elapsed());

    let start = Instant::now();
    let _sbvh = Sbvh::new(&polygons);
    println!("Built SBVH in {:?}", start.elapsed());
}
