import ctypes

# 加载库
lib = ctypes.CDLL('./PRE.so')

# 设置函数原型
lib.prediction.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_longlong]
lib.prediction.restype = ctypes.POINTER(ctypes.c_double)

# 创建数组
rows, cols = 101, 5
array_ptr = lib.prediction(2.0,2.0,1.0,1.0,10000)

# 将一维数组转换为二维列表
array = [[array_ptr[i * cols + j] for j in range(cols)] for i in range(rows)]

# 释放数组
lib.free2DArray(array_ptr)
print(array)
