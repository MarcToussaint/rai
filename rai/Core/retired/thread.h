// macro for a most standard declaration of a module
#define BEGIN_MODULE(name) \
  struct name : Thread { \
    struct s##name *s; \
    name(): Thread(#name), s(NULL) {} \
    virtual void open(); \
    virtual void step(); \
    virtual void close();

#define END_MODULE() };
