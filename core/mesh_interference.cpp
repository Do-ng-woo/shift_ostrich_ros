// Triangle BVH queries against original STL geometry, including adjacent links.
#include <fcl/fcl.h>
#include <memory>
#include <vector>

using Mesh = fcl::BVHModel<fcl::OBBRSSd>;

extern "C" void* shift_mesh_create(const double* xyz, int vertex_count) {
  auto mesh = new Mesh;
  std::vector<fcl::Vector3d> vertices;
  std::vector<fcl::Triangle> triangles;
  for (int i=0; i<vertex_count; ++i)
    vertices.emplace_back(xyz[3*i], xyz[3*i+1], xyz[3*i+2]);
  for (int i=0; i+2<vertex_count; i+=3) triangles.emplace_back(i,i+1,i+2);
  mesh->beginModel(); mesh->addSubModel(vertices, triangles); mesh->endModel();
  return mesh;
}

extern "C" void shift_mesh_destroy(void* mesh) { delete static_cast<Mesh*>(mesh); }

static fcl::Transform3d pose(const double* values) {
  fcl::Transform3d t = fcl::Transform3d::Identity();
  for (int row=0; row<3; ++row) {
    for (int col=0; col<3; ++col) t.linear()(row,col) = values[4*row+col];
    t.translation()[row] = values[4*row+3];
  }
  return t;
}

extern "C" int shift_mesh_contact(void* a, const double* ta, void* b, const double* tb,
                                    double* point) {
  fcl::CollisionRequestd request(1024, true);
  fcl::CollisionResultd result;
  fcl::collide(static_cast<Mesh*>(a), pose(ta), static_cast<Mesh*>(b), pose(tb), request, result);
  if (!result.isCollision()) return 0;
  auto ma = static_cast<Mesh*>(a);
  auto mb = static_cast<Mesh*>(b);
  const auto pa = pose(ta), pb = pose(tb);
  for (size_t index=0; index<result.numContacts(); ++index) {
    const auto& contact = result.getContact(index);
    for (int i=0; i<3; ++i) point[i] = contact.pos[i];
    if (contact.b1 < 0 || contact.b2 < 0) continue;
    const bool same_order = contact.o1 == ma;
    const auto ia = ma->tri_indices[same_order ? contact.b1 : contact.b2];
    const auto ib = mb->tri_indices[same_order ? contact.b2 : contact.b1];
    fcl::Vector3d va[3], vb[3];
    for (int i=0; i<3; ++i) { va[i] = pa*ma->vertices[ia[i]]; vb[i] = pb*mb->vertices[ib[i]]; }
    const auto na = (va[1]-va[0]).cross(va[2]-va[0]).normalized().eval();
    const auto nb = (vb[1]-vb[0]).cross(vb[2]-vb[0]).normalized().eval();
    double amin=1e9, amax=-1e9, bmin=1e9, bmax=-1e9;
    for (int i=0; i<3; ++i) {
      double da=nb.dot(va[i]-vb[0]), db=na.dot(vb[i]-va[0]);
      amin=std::min(amin,da); amax=std::max(amax,da);
      bmin=std::min(bmin,db); bmax=std::max(bmax,db);
    }
    // A collided triangle pair straddles both planes: transverse crossing,
    // not just coplanar contact. Epsilon is 0.1 micrometre in metre units.
    if (amin < -1e-7 && amax > 1e-7 && bmin < -1e-7 && bmax > 1e-7) return 2;
  }
  return 1;
}
