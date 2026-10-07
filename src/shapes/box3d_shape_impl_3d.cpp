#include "box3d_shape_impl_3d.hpp"

#include "../objects/box3d_shaped_object_impl_3d.hpp"

#include <vector>

void Box3DShapeImpl3D::add_owner(Box3DShapedObjectImpl3D* p_owner) {
	owners.insert(p_owner);
}

void Box3DShapeImpl3D::remove_owner(Box3DShapedObjectImpl3D* p_owner) {
	owners.erase(p_owner);
}

void Box3DShapeImpl3D::update_data(const Variant& p_data) {
	if (get_type() == PhysicsServer3D::SHAPE_CUSTOM) {
		set_data(p_data);
		return;
	}
	std::vector<Box3DShapedObjectImpl3D*> attached;
	for (Box3DShapedObjectImpl3D* owner : owners) {
		attached.push_back(owner);
	}
	for (Box3DShapedObjectImpl3D* owner : attached) {
		owner->release_shape_instances(this);
	}
	set_data(p_data);
	for (Box3DShapedObjectImpl3D* owner : attached) {
		owner->restore_shape_instances(this);
	}
}

void Box3DShapeImpl3D::detach_from_owners() {
	std::vector<Box3DShapedObjectImpl3D*> attached;
	for (Box3DShapedObjectImpl3D* owner : owners) {
		attached.push_back(owner);
	}
	for (Box3DShapedObjectImpl3D* owner : attached) {
		owner->detach_shape(this);
	}
}
