    1、导入fdsst
    2、更新SSE2NEON.h文件
    3、arm平台，注释掉sse.hpp里的emmintrin.h,再将SSE2NEON.h include进来
    4、导入fdsst算法的源码里用到了SSE汇编指令加速，用arm的NEON指令集实现SSE的同名函数
    Q:error: "NEON support not enabled"
    A:arguments "-DANDROID_ARM_NEON=TRUE", "-DANDROID_TOOLCHAIN=clang"


https://blog.csdn.net/u010097644/article/details/70881102

https://blog.csdn.net/u010097644/article/details/70881163

https://github.com/jratcliff63367/sse2neon