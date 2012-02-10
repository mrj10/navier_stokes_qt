HEADERS       = fluidwidget.h \
                renderthread.h \
								fluid_update.h
SOURCES       = main.cpp \
                fluidwidget.cpp \
                renderthread.cpp

CONFIG += release

QMAKE_CXXFLAGS_RELEASE += -Ofast -mtune=native -flto

unix:!mac:!symbian:!vxworks:LIBS += -lm

# install
target.path = /media/d2/sw/fluid
sources.files = $$SOURCES $$HEADERS $$RESOURCES $$FORMS fluid.pro
sources.path = /media/d2/sw/fluid
INSTALLS += target sources

symbian: include($$QT_SOURCE_TREE/examples/symbianpkgrules.pri)
