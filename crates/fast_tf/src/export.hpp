#pragma once

// fast_tf library export macros
#ifdef FAST_TF_EXPORTS
# define FAST_TF_API __attribute__((visibility("default")))
#else
# define FAST_TF_API
#endif
