#pragma once

// scheduler library export macros
#ifdef SCHEDULER_EXPORTS
# define SCHEDULER_API __attribute__((visibility("default")))
#else
# define SCHEDULER_API
#endif
