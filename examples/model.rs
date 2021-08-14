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
    println!("Loading model...");
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

                    let mut min: Point3<f32> = Point3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    };
                    let mut max: Point3<f32> = Point3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    };
                    for point in &points {
                        min.x = min.x.min(point.x);
                        min.y = min.y.min(point.y);
                        min.z = min.z.min(point.z);

                        max.x = max.x.max(point.x);
                        max.y = max.y.max(point.y);
                        max.z = max.z.max(point.z);
                    }

                    Polygon {
                        points,
                        bounding_box: Aabb::new(min, max),
                    }
                })
            })
        })
        .collect::<Vec<_>>();

    println!("Building SBVH...");
    let _sbvh = Sbvh::new(&polygons);
}
