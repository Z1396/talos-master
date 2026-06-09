#pragma once

// quanta library export macros
#ifdef QUANTA_EXPORTS
# define QUANTA_API __attribute__((visibility("default")))
#else
# define QUANTA_API
#endif
