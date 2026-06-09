#pragma once

// hikcamera library export macros
#ifdef HIKCAMERA_EXPORTS
# define HIKCAMERA_API __attribute__((visibility("default")))
#else
# define HIKCAMERA_API
#endif
