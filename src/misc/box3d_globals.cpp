#include "box3d_globals.hpp"

#include "box3d_cpu_topology.hpp"

#include <box3d/constants.h>

#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <algorithm>

using namespace godot;

namespace {
const char *WORKER_COUNT_SETTING = "physics/box3d/worker_count";
const char *DIAGNOSTICS_SETTING = "physics/box3d/diagnostics";
// Terra: contact softness. Box3D pushes overlapping bodies apart at up to contact_speed, which on a
// body of thousands of tonnes is an enormous impulse; Godot and Jolt recover gently.
const char *CONTACT_HERTZ_SETTING = "physics/box3d/contact_hertz";
const char *CONTACT_DAMPING_SETTING = "physics/box3d/contact_damping_ratio";
const char *CONTACT_SPEED_SETTING = "physics/box3d/contact_speed";

void add_float_setting(ProjectSettings *p_settings, const char *p_name, double p_default, const char *p_range) {
	if (!p_settings->has_setting(p_name)) {
		p_settings->set_setting(p_name, p_default);
	}
	p_settings->set_initial_value(p_name, p_default);
	p_settings->set_restart_if_changed(p_name, true);
	Dictionary info;
	info["name"] = p_name;
	info["type"] = Variant::FLOAT;
	info["hint"] = PROPERTY_HINT_RANGE;
	info["hint_string"] = p_range;
	p_settings->add_property_info(info);
}
} // namespace

void box3d_initialize() {
	ProjectSettings *settings = ProjectSettings::get_singleton();
	if (!settings->has_setting(WORKER_COUNT_SETTING)) {
		settings->set_setting(WORKER_COUNT_SETTING, 0);
	}
	settings->set_initial_value(WORKER_COUNT_SETTING, 0);
	// Spaces capture the count at creation, so a change only applies after a restart.
	settings->set_restart_if_changed(WORKER_COUNT_SETTING, true);
	Dictionary info;
	info["name"] = WORKER_COUNT_SETTING;
	info["type"] = Variant::INT;
	info["hint"] = PROPERTY_HINT_RANGE;
	info["hint_string"] = "0,32,1";
	settings->add_property_info(info);

	if (!settings->has_setting(DIAGNOSTICS_SETTING)) {
		settings->set_setting(DIAGNOSTICS_SETTING, false);
	}
	settings->set_initial_value(DIAGNOSTICS_SETTING, false);
	Dictionary diagnostics_info;
	diagnostics_info["name"] = DIAGNOSTICS_SETTING;
	diagnostics_info["type"] = Variant::BOOL;
	settings->add_property_info(diagnostics_info);

	// Box3D's own defaults (b3DefaultWorldDef).
	add_float_setting(settings, CONTACT_HERTZ_SETTING, 30.0, "1,240,0.1");
	add_float_setting(settings, CONTACT_DAMPING_SETTING, 10.0, "0,100,0.1");
	add_float_setting(settings, CONTACT_SPEED_SETTING, 3.0, "0.01,100,0.01,suffix:m/s");
}

void box3d_deinitialize() {
	// No global Box3D shutdown is required.
}

int box3d_worker_count() {
	static const int count = []() {
		const int setting = (int)ProjectSettings::get_singleton()->get_setting_with_override(
				WORKER_COUNT_SETTING);
		return setting > 0 ? std::clamp(setting, 1, B3_MAX_WORKERS) : box3d_default_worker_count();
	}();
	return count;
}

float box3d_contact_hertz() {
	return (float)(double)ProjectSettings::get_singleton()->get_setting_with_override(CONTACT_HERTZ_SETTING);
}

float box3d_contact_damping_ratio() {
	return (float)(double)ProjectSettings::get_singleton()->get_setting_with_override(CONTACT_DAMPING_SETTING);
}

float box3d_contact_speed() {
	return (float)(double)ProjectSettings::get_singleton()->get_setting_with_override(CONTACT_SPEED_SETTING);
}

bool box3d_diagnostics_enabled() {
	static const bool enabled = (bool)ProjectSettings::get_singleton()->get_setting_with_override(DIAGNOSTICS_SETTING);
	return enabled;
}
