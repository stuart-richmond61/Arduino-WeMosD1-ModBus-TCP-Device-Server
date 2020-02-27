
#define _PRINTF_BUFFER_LENGTH_  132

#if 1
static char _pf_buffer_[_PRINTF_BUFFER_LENGTH_];
#else
extern char _pf_buffer_[_PRINTF_BUFFER_LENGTH_];
#endif

#define printf(_obj_,a,...)                                                   \
  do{                                                                   \
    snprintf(_pf_buffer_, sizeof(_pf_buffer_), a, ##__VA_ARGS__);       \
    _obj_.print(_pf_buffer_);                                    \
  }while(0)

#define printfn(_obj_,a,...)                                                  \
  do{                                                                   \
    snprintf(_pf_buffer_, sizeof(_pf_buffer_), a"\r\n", ##__VA_ARGS__); \
    _obj_.print(_pf_buffer_);                                    \
  }while(0)

