QT += quick quick3d network concurrent

CONFIG += qmltypes
QML_IMPORT_NAME = CustomGeometryExample
QML_IMPORT_MAJOR_VERSION = 1

target.path = $$[QT_INSTALL_EXAMPLES]/quick3d/customgeometry
INSTALLS += target

SOURCES += \
    main.cpp \
    examplegeometry.cpp \
    pointcloudfileloader.cpp \
    pointcloudgeometry.cpp \
    pointcloudprotocol.cpp \
    routegeometry.cpp \
    udptransport.cpp

HEADERS += \
    examplegeometry.h \
    pointcloudfileloader.h \
    pointcloudgeometry.h \
    pointcloudprotocol.h \
    routegeometry.h \
    udptransport.h

qml_resources.files = \
    qmldir \
    Main.qml \
    TorusMesh.qml \
    qt_logo_rect.png

qml_resources.prefix = /qt/qml/CustomGeometryExample/

RESOURCES += qml_resources

OTHER_FILES += \
    doc/src/*.*
