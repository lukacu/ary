#include <gdnative_api_struct.gen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camera.h"
#include "ary.h"

typedef struct camera_data_struct {
	void* camera;
    godot_pool_byte_array buffer;
    unsigned long counter;
} camera_data_struct;

typedef struct ar_data_struct {
	void* ary;
    int detect_anchors;
    int* controllers;
    godot_transform camera;
} ar_data_struct;


const godot_gdnative_core_api_struct *api = NULL;
const godot_gdnative_ext_nativescript_api_struct *nativescript_api = NULL;
const godot_gdnative_ext_arvr_api_struct *arvr_api = NULL;

GDCALLINGCONV void *_camera_constructor(godot_object *p_instance, void *p_method_data);
GDCALLINGCONV void _camera_destructor(godot_object *p_instance, void *p_method_data, void *p_user_data);
godot_variant _camera_get_image(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args);
godot_variant _camera_get_width(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args);
godot_variant _camera_get_height(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args);
godot_variant _camera_open(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args);
godot_variant _camera_set_default(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args);

GDCALLINGCONV void* _ar_constructor(godot_object *);
GDCALLINGCONV void _ar_destructor(void *);
godot_string _ar_get_name(const void *);
godot_int _ar_get_capabilities(const void *);
godot_bool _ar_get_anchor_detection_is_enabled(const void *);
void _ar_set_anchor_detection_is_enabled(void *, godot_bool);
godot_bool _ar_is_stereo(const void *);
godot_bool _ar_is_initialized(const void *);
godot_bool _ar_initialize(void *);
void _ar_uninitialize(void *);
godot_vector2 _ar_get_render_targetsize(const void *);
godot_transform _ar_get_transform_for_eye(void *, godot_int, godot_transform *);
void _ar_fill_projection_for_eye(void *, godot_real *, godot_int, godot_real, godot_real, godot_real);
void _ar_commit_for_eye(void *, godot_int, godot_rid *, godot_rect2 *);
void _ar_process(void *);


void _convert_ary_transform(const ary_transform* at, godot_transform * gt) {

    godot_vector3 x, y, z, t;

    api->godot_vector3_new(&x, at->data[0], at->data[1], at->data[2]);
    api->godot_vector3_new(&y, at->data[3], at->data[4], at->data[5]);
    api->godot_vector3_new(&z, at->data[6], at->data[7], at->data[8]);
    api->godot_vector3_new(&t, at->data[9], at->data[10], at->data[11]);

    api->godot_transform_new_with_axis_origin(gt, &x, &y, &z, &t);

}

void GDN_EXPORT godot_gdnative_init(godot_gdnative_init_options *p_options) {
	api = p_options->api_struct;

	// now find our extensions
	for (int i = 0; i < api->num_extensions; i++) {
		switch (api->extensions[i]->type) {
			case GDNATIVE_EXT_ARVR: {
				arvr_api = (godot_gdnative_ext_arvr_api_struct *)api->extensions[i];
            }; break;
			case GDNATIVE_EXT_NATIVESCRIPT: {
				nativescript_api = (godot_gdnative_ext_nativescript_api_struct *)api->extensions[i];
			}; break;
			default: break;
		};
	};
}

godot_arvr_interface_gdnative ar_interface;

void GDN_EXPORT godot_gdnative_terminate(godot_gdnative_terminate_options *p_options) {

    camera_delete_all();

	api = NULL;
	nativescript_api = NULL;
}

void GDN_EXPORT godot_nativescript_init(void *p_handle) {

	godot_instance_create_func create = { NULL, NULL, NULL };
	create.create_func = &_camera_constructor;

	godot_instance_destroy_func destroy = { NULL, NULL, NULL };
	destroy.destroy_func = &_camera_destructor;

	nativescript_api->godot_nativescript_register_class(p_handle, "Camera", "Reference", create, destroy);

	godot_method_attributes attributes = { GODOT_METHOD_RPC_MODE_DISABLED };

	godot_instance_method get_image = { NULL, NULL, NULL };
	get_image.method = &_camera_get_image;

	nativescript_api->godot_nativescript_register_method(p_handle, "Camera", "get_image", attributes, get_image);

	godot_instance_method open = { NULL, NULL, NULL };
	open.method = &_camera_open;

	nativescript_api->godot_nativescript_register_method(p_handle, "Camera", "open", attributes, open);

	godot_instance_method get_width = { NULL, NULL, NULL };
	get_width.method = &_camera_get_width;

	nativescript_api->godot_nativescript_register_method(p_handle, "Camera", "get_width", attributes, get_width);


	godot_instance_method get_height = { NULL, NULL, NULL };
	get_height.method = &_camera_get_height;

	nativescript_api->godot_nativescript_register_method(p_handle, "Camera", "get_height", attributes, get_height);

    godot_instance_method set_default = { NULL, NULL, NULL };
	set_default.method = &_camera_set_default;

	nativescript_api->godot_nativescript_register_method(p_handle, "Camera", "set_default", attributes, set_default);


    ar_interface.constructor = &_ar_constructor;
    ar_interface.destructor = &_ar_destructor;
    ar_interface.get_name = &_ar_get_name;
    ar_interface.get_capabilities = &_ar_get_capabilities;
    ar_interface.get_anchor_detection_is_enabled = &_ar_get_anchor_detection_is_enabled;
    ar_interface.is_stereo = &_ar_is_stereo;
    ar_interface.is_initialized = &_ar_is_initialized;
    ar_interface.initialize = &_ar_initialize;
    ar_interface.uninitialize = &_ar_uninitialize;
    ar_interface.get_render_targetsize = &_ar_get_render_targetsize;
    ar_interface.get_transform_for_eye = &_ar_get_transform_for_eye;
    ar_interface.fill_projection_for_eye = &_ar_fill_projection_for_eye;
    ar_interface.commit_for_eye = &_ar_commit_for_eye;
    ar_interface.process = &_ar_process;

    arvr_api->godot_arvr_register_interface(&ar_interface);

}

GDCALLINGCONV void *_camera_constructor(godot_object *p_instance, void *p_method_data) {

    camera_data_struct *user_data = api->godot_alloc(sizeof(camera_data_struct));

	user_data->camera = NULL;
    api->godot_pool_byte_array_new(&user_data->buffer);
    user_data->counter = 0;

    return user_data;

}

GDCALLINGCONV void _camera_destructor(godot_object *p_instance, void *p_method_data, void *p_user_data) {

    camera_data_struct *user_data = (camera_data_struct*) p_user_data;

    if (user_data->camera) {
        camera_delete(user_data->camera);
        user_data->camera = NULL;
    }

    api->godot_pool_byte_array_destroy(&user_data->buffer);
    api->godot_free(p_user_data);
}

godot_variant _camera_set_default(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args) {

    int camera_id = 0, success;
    int width, height, ms;
    godot_variant res;

    if (p_num_args > 0) {
        camera_id = api->godot_variant_as_int(p_args[0]);
    }

    camera_set_default(camera_id);

    api->godot_variant_new_bool(&res, 1);
	return res;
}

godot_variant _camera_open(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args) {

    int camera_id = -1, success;
    int width, height, ms;
    godot_variant res;

    camera_data_struct *user_data = (camera_data_struct*) p_user_data;

    if (p_num_args > 0) {
        camera_id = api->godot_variant_as_int(p_args[0]);
    }

    if (!user_data) {
        api->godot_variant_new_bool(&res, GODOT_FALSE);
	    return res;
    }

    if (user_data->camera) {
        camera_delete(user_data->camera);
    }

    user_data->camera = camera_create(camera_id);

    if (!user_data->camera) {
        api->godot_variant_new_bool(&res, GODOT_FALSE);
	    return res;
    }

    width = camera_get_width(user_data->camera);
    height = camera_get_height(user_data->camera);

    ms = (width < height) ? height : width;
    api->godot_pool_byte_array_resize(&user_data->buffer, ms * ms * 3);

    api->godot_variant_new_bool(&res, 1);
	return res;
}

godot_variant _camera_get_image(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args) {

    int updated = 0;

    godot_variant res;
	camera_data_struct * user_data = (camera_data_struct *) p_user_data;
    godot_pool_byte_array_write_access* writer;

    writer = api->godot_pool_byte_array_write(&user_data->buffer);
    uint8_t* data = api->godot_pool_byte_array_write_access_ptr(writer);
    updated = camera_get_image(user_data->camera, data, &user_data->counter);
    api->godot_pool_byte_array_write_access_destroy(writer);


    if (updated) {
        api->godot_variant_new_pool_byte_array(&res, &user_data->buffer);
    } else {
        api->godot_variant_new_bool(&res, GODOT_FALSE);
    }

	return res;
}

godot_variant _camera_get_width(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args) {

    godot_variant res;
	camera_data_struct * user_data = (camera_data_struct *) p_user_data;

    api->godot_variant_new_int(&res, camera_get_width(user_data->camera));
	return res;
}

godot_variant _camera_get_height(godot_object *p_instance, void *p_method_data, void *p_user_data, int p_num_args, godot_variant **p_args) {

    godot_variant res;
	camera_data_struct * user_data = (camera_data_struct *) p_user_data;

    api->godot_variant_new_int(&res, camera_get_height(user_data->camera));
	return res;
}


GDCALLINGCONV void* _ar_constructor(godot_object *p_user_data) {

    ar_data_struct *user_data = api->godot_alloc(sizeof(ar_data_struct));

	user_data->ary = NULL;
    user_data->detect_anchors = 1;
    user_data->controllers = NULL;

    api->godot_transform_new_identity(&user_data->camera);

    return user_data;
}


GDCALLINGCONV void _ar_destructor(void *p_user_data) {

    ar_data_struct *user_data = (ar_data_struct*) p_user_data;

    if (user_data->ary) {
        ary_delete(user_data->ary);
        user_data->ary = NULL;
    }

    if (user_data->controllers) {
        api->godot_free(user_data->controllers);
        user_data->controllers = NULL;
    }

    api->godot_free(p_user_data);

}

godot_string _ar_get_name(const void *p_user_data) {

    godot_string name;

    api->godot_string_new(&name);
    api->godot_string_parse_utf8(&name, "ARy");

    return name;

}

godot_int _ar_get_capabilities(const void *p_user_data) {

    return 4 + 1; //  ARVR_MONO & ARVR_AR

}

godot_bool _ar_get_anchor_detection_is_enabled(const void *p_user_data) {

    return GODOT_TRUE;

}

void _ar_set_anchor_detection_is_enabled(void *p_user_data, godot_bool enabled) {

    ar_data_struct *user_data = (ar_data_struct*) p_user_data;

    user_data->detect_anchors = enabled;

}

godot_bool _ar_is_stereo(const void *p_user_data) {

    return GODOT_FALSE;

}

godot_bool _ar_is_initialized(const void *p_user_data) {

    ar_data_struct *user_data = (ar_data_struct*) p_user_data;

    if (!user_data) return GODOT_FALSE;

    return user_data->ary ? GODOT_TRUE : GODOT_FALSE;

}

godot_bool _ar_initialize(void *p_user_data) {

    int i;

    ar_data_struct *user_data = (ar_data_struct*) p_user_data;

    if (!user_data->ary)
        user_data->ary = ary_create();

    user_data->controllers = (int*) api->godot_alloc(sizeof(int) * ary_anchor_count(user_data->ary));

    for (i = 1; i < ary_anchor_count(user_data->ary); i++) {

        user_data->controllers[i] = arvr_api->godot_arvr_add_controller((char*)ary_anchor_name(user_data->ary, i), 0, GODOT_TRUE, GODOT_TRUE);

        printf("Registered marker %d with id %d\n", i, user_data->controllers[i]);

    }

    return GODOT_TRUE;

}


void _ar_uninitialize(void *p_user_data) {

    int i;

    ar_data_struct *user_data = (ar_data_struct*) p_user_data;

    if (!user_data) return;

    for (i = 1; i < ary_anchor_count(user_data->ary); i++) {

        arvr_api->godot_arvr_remove_controller(user_data->controllers[i]);

    }

    if (user_data->controllers) {
        api->godot_free(user_data->controllers);
        user_data->controllers = NULL;
    }

    if (user_data->ary) {
        ary_delete(user_data->ary);
        user_data->ary = NULL;
    }

}

godot_vector2 _ar_get_render_targetsize(const void *p_user_data) {

    godot_vector2 size;

    ar_data_struct *user_data = (ar_data_struct*) p_user_data;

    if (!user_data || !user_data->ary) {
        api->godot_vector2_new(&size, 0, 0);
        return size;
    }

    ary_size asize = ary_camera_size(user_data->ary);

    api->godot_vector2_new(&size, asize.width, asize.height);

    return size;

}

godot_transform _ar_get_transform_for_eye(void *p_user_data, godot_int eye, godot_transform *transform) {

    godot_transform reference = arvr_api->godot_arvr_get_reference_frame();

    ar_data_struct *user_data = (ar_data_struct*) p_user_data;

    if (!user_data || !user_data->ary) {
        return *transform;
    }

    godot_transform eyet = api->godot_transform_operator_multiply(&reference, &user_data->camera);

    eyet = api->godot_transform_operator_multiply(transform, &eyet);

    return eyet;

}

void _ar_fill_projection_for_eye(void *p_user_data, godot_real * a, godot_int eye, godot_real aspect, godot_real near, godot_real far) {


}

void _ar_commit_for_eye(void *p_user_data, godot_int eye, godot_rid * rid, godot_rect2 * rect) {


    arvr_api->godot_arvr_blit(eye, rid, rect);

/*

// helper functions to access ARVRServer data
godot_real GDAPI godot_arvr_get_worldscale();
godot_transform GDAPI godot_arvr_get_reference_frame();

// helper functions for rendering
void GDAPI godot_arvr_blit(godot_int p_eye, godot_rid *p_render_target, godot_rect2 *p_rect);
godot_int GDAPI godot_arvr_get_texid(godot_rid *p_render_target);

*/
}

void _ar_process(void *p_user_data) {

    int i, state;

    ar_data_struct *user_data = (ar_data_struct*) p_user_data;

    if (!ary_process(user_data->ary)) return;

    if (ary_anchor_active(user_data->ary, 0)) {

        ary_transform at = ary_anchor_transform(user_data->ary, 0);

        _convert_ary_transform(&at, &user_data->camera);

    }

    godot_transform gt;
    ary_transform at;

    godot_transform inv = api->godot_transform_inverse(&user_data->camera);

    for (i = 1; i < ary_anchor_count(user_data->ary); i++) {

        if (ary_anchor_active(user_data->ary, i)) {

            at = ary_anchor_itransform(user_data->ary, i);

            _convert_ary_transform(&at, &gt);

            gt = api->godot_transform_operator_multiply(&user_data->camera, &gt);


            arvr_api->godot_arvr_set_controller_transform(user_data->controllers[i], &gt, GODOT_TRUE, GODOT_TRUE);

        } else {


        }



    }



}


