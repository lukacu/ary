
#ifndef ARY_C_API_H
#define ARY_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
Transform layout:
0 3 6 9
1 4 7 10
2 5 8 11
*/

typedef struct ary_transform { float data[12]; } ary_transform;

typedef struct ary_size { int width; int height; } ary_size;


void* ary_create();

void ary_delete(void *obj);

int ary_process(void *obj);

int ary_anchor_count(void *obj);

int ary_anchor_active(void *obj, int id);

int ary_anchor_type(void *obj, int id);

const char* ary_anchor_name(void *obj, int id);

ary_transform ary_anchor_transform(void *obj, int id);

ary_transform ary_anchor_itransform(void *obj, int id);

ary_size ary_camera_size(void *obj);

#ifdef __cplusplus
}
#endif


#endif
