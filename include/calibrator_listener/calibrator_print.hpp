#define _DEBUG

#ifdef _DEBUG
#define debug_printf printf
#else
#define debug_printf /\
/debug_printf
#endif

#ifdef _DEBUG
#define debug_cout std::cout
#else
#define debug_cout 0 && std::cout
#endif

#ifdef _DEBUG
#define debug_info ROS_INFO
#else
#define debug_info /\
/debug_info
#endif

#ifdef _DEBUG
#define debug_error ROS_ERROR
#else
#define debug_error /\
/debug_error
#endif

#ifdef _DEBUG
#define debug_debug ROS_DEBUG
#else
#define debug_debug /\
/debug_debug
#endif

#ifdef _DEBUG
#define debug_warn ROS_WARN
#else
#define debug_warn /\
/debug_warn
#endif