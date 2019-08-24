# gflag
- 头文件 #include "gflags.h"


example
```cpp
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");

if (FLAGS_collect_metrics == false) {
  // TODO(@someone)
}
```
