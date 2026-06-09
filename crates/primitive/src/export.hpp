#pragma once

// primitive library export macros
#ifdef PRIMITIVE_EXPORTS
# define PRIMITIVE_API __attribute__((visibility("default")))
#else
# define PRIMITIVE_API
#endif
