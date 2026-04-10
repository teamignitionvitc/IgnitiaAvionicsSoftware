// guva_hw837.h
// Header for GUVA HW-837 UV sensor driver

#ifndef GUV_HW837_H
#define GUV_HW837_H

#ifdef __cplusplus
extern "C" {
#endif

void guva_hw837_init(void);
float guva_hw837_read_uv(void);

#ifdef __cplusplus
}
#endif

#endif // GUV_HW837_H
