# ros_rs_wrapper

## Use cv_bridge with opencv4:

Change cv_bridgeConfig.cmake at line 94, add /usr/include/opencv4, which becomes: 

```
if(NOT "include;/usr/include;/usr/include/opencv;/usr/include/opencv4" ...) 
```

And also change the include files from #include <cv.h> to #include <opencv2/opencv.hpp>

update: 
not add but replace "/usr/include/opencv" with "/user/include/opencv4" at line 94 and line 96, otherwise, the error "usr/include/opencv does exist" will appear
