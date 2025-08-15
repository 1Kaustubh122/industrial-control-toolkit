#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#define ICTK_C_ABI_VER 1

// // Opaque handles
typedef struct ictk_ctx_t      ictk_ctx_t;
typedef struct ictk_controller ictk_controller;

// // NOTE: Functions intentionally deferred to later to avoid ABI churn.

#ifdef __cplusplus
}
#endif
