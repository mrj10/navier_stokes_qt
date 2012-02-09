HEADERS       = fluidwidget.h \
                renderthread.h
SOURCES       = main.cpp \
                fluidwidget.cpp \
                renderthread.cpp

unix:!mac:!symbian:!vxworks:LIBS += -lm

# install
target.path = /media/d2/sw/fluid
sources.files = $$SOURCES $$HEADERS $$RESOURCES $$FORMS fluid.pro
sources.path = /media/d2/sw/fluid
INSTALLS += target sources

symbian: include($$QT_SOURCE_TREE/examples/symbianpkgrules.pri)
