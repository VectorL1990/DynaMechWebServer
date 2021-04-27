#需要生成可执行程序的Makefile

#程序根目录
top_srcdir =.

#目标程序名
TARGET = MatrixExample

CPP_FILES = $(shell ls *.cpp)

C_FILES = $(-shell ls *.c)

SRCS = $(CPP_FILES) $(C_FILES)

BASE = $(basename $(SRCS))

OBJS = $(addsuffix .o, $(BASE))

DEPS = $(addsuffix .d, $(addprefix dep/,$(BASE)))

#包含公共Make规则
include $(top_srcdir)/makeinclude/Make.rules

#额外需要包含的头文件的目录位置
INCLUDEDIR := $(INCLUDEDIR)\
	-I$(top_srcdir)/include\

#所有要包含的静态库的名称
LIBS := -lopenblas

#设置目标程序依赖的.o文件
$(TARGET):$(OBJS) $(LIBS)
	-rm -f $@
	$(CXX) -o $(TARGET) $(INCLUDEDIR) $(LDFLAGS) $(OBJS) $(LIBS)

#objects = MatrixExample.o
#MatrixExample : $(objects) $(LIBS)
#	#-rm -f $@
#	$(CXX) -o $(TARGET) $(INCLUDEDIR) $(LDFLAGS) $(objects) $(LIBS)

MatrixExample.o : MatrixExample.cpp
	g++ $(INCLUDEDIR) -c MatrixExample.cpp

clean:
	rm *.o
