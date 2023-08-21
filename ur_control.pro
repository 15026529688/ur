QT       += core gui concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    ballkeep_thread.cpp \
    conekeep_thread.cpp \
    freemove_thread.cpp \
    linekeep_thread.cpp \
    main.cpp \
    my_thread.cpp \
    newplanekeep_thread.cpp \
    planekeep_thread.cpp \
    robot_data.cpp \
    testmainwindow.cpp

HEADERS += \
    ballkeep_thread.h \
    conekeep_thread.h \
    freemove_thread.h \
    linekeep_thread.h \
    my_thread.h \
    newplanekeep_thread.h \
    planekeep_thread.h \
    robot_data.h \
    testmainwindow.h

FORMS += \
    testmainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#win32:CONFIG(release, debug|release): LIBS += -LD:/ur_rtde-master/out/build/x64-Release/ -lrtde
#else:win32:CONFIG(debug, debug|release): LIBS += -LD:/ur_rtde-master/out/build/x64-Release/ -lrtded
#else:unix: LIBS += -LD:/ur_rtde-master/out/build/x64-Release/ -lrtde


##DEPENDPATH += D:/ur_rtde-master/out/build/x64-Release
#INCLUDEPATH +=D:/ur_rtde-master/include
#INCLUDEPATH +=E:/C++Library/boost_1_81_0
#INCLUDEPATH +=E:/C++Library/eigen-3.4.0

win32:CONFIG(release, debug|release): LIBS += -LD:/rtde/ur_rtde-master/ur_rtde-master/out/build/x64-Release/ -lrtde
else:win32:CONFIG(debug, debug|release): LIBS += -LD:/rtde/ur_rtde-master/ur_rtde-master/out/build/x64-Release/ -lrtded
else:unix: LIBS += -LD:/rtde/ur_rtde-master/ur_rtde-master/out/build/x64-Release/ -lrtde

win32:CONFIG(release, debug|release): LIBS += -LD:/MITK/Lib
INCLUDEPATH +=D:/rtde/ur_rtde-master/ur_rtde-master/include
INCLUDEPATH +=D:/rtde/boost_1_81_0
INCLUDEPATH +=D:/rtde/eigen-3.4.0
INCLUDEPATH +=D:/MITK/Include/vtk-9.0

LIBS+=-lvtkChartsCore-9.0
LIBS+=-lvtkCommonColor-9.0
LIBS+=-lvtkCommonComputationalGeometry-9.0
LIBS+=-lvtkCommonCore-9.0
LIBS+=-lvtkCommonDataModel-9.0
LIBS+=-lvtkCommonExecutionModel-9.0
LIBS+=-lvtkCommonMath-9.0
LIBS+=-lvtkCommonMisc-9.0
LIBS+=-lvtkCommonSystem-9.0
LIBS+=-lvtkCommonTransforms-9.0
LIBS+=-lvtkDICOM-9.0
LIBS+=-lvtkDICOMParser-9.0
LIBS+=-lvtkDomainsChemistry-9.0
LIBS+=-lvtkdoubleconversion-9.0
LIBS+=-lvtkexodusII-9.0
LIBS+=-lvtkexpat-9.0
LIBS+=-lvtkFiltersAMR-9.0
LIBS+=-lvtkFiltersCore-9.0
LIBS+=-lvtkFiltersExtraction-9.0
LIBS+=-lvtkFiltersFlowPaths-9.0
LIBS+=-lvtkFiltersGeneral-9.0
LIBS+=-lvtkFiltersGeneric-9.0
LIBS+=-lvtkFiltersGeometry-9.0
LIBS+=-lvtkFiltersHybrid-9.0
LIBS+=-lvtkFiltersHyperTree-9.0
LIBS+=-lvtkFiltersImaging-9.0
LIBS+=-lvtkFiltersModeling-9.0
LIBS+=-lvtkFiltersParallel-9.0
LIBS+=-lvtkFiltersParallelImaging-9.0
LIBS+=-lvtkFiltersPoints-9.0
LIBS+=-lvtkFiltersProgrammable-9.0
LIBS+=-lvtkFiltersSelection-9.0
LIBS+=-lvtkFiltersSMP-9.0
LIBS+=-lvtkFiltersSources-9.0
LIBS+=-lvtkFiltersStatistics-9.0
LIBS+=-lvtkFiltersTexture-9.0
LIBS+=-lvtkFiltersTopology-9.0
LIBS+=-lvtkFiltersVerdict-9.0
LIBS+=-lvtkfreetype-9.0
LIBS+=-lvtkGeovisCore-9.0
LIBS+=-lvtkgl2ps-9.0
LIBS+=-lvtkglew-9.0
LIBS+=-lvtkGUISupportQt-9.0
LIBS+=-lvtkGUISupportQtSQL-9.0
LIBS+=-lvtkhdf5-9.0
LIBS+=-lvtkhdf5_hl-9.0
LIBS+=-lvtkImagingColor-9.0
LIBS+=-lvtkImagingCore-9.0
LIBS+=-lvtkImagingFourier-9.0
LIBS+=-lvtkImagingGeneral-9.0
LIBS+=-lvtkImagingHybrid-9.0
LIBS+=-lvtkImagingMath-9.0
LIBS+=-lvtkImagingMorphological-9.0
LIBS+=-lvtkImagingSources-9.0
LIBS+=-lvtkImagingStatistics-9.0
LIBS+=-lvtkImagingStencil-9.0
LIBS+=-lvtkInfovisCore-9.0
LIBS+=-lvtkInfovisLayout-9.0
LIBS+=-lvtkInteractionImage-9.0
LIBS+=-lvtkInteractionStyle-9.0
LIBS+=-lvtkInteractionWidgets-9.0
LIBS+=-lvtkIOAMR-9.0
LIBS+=-lvtkIOAsynchronous-9.0
LIBS+=-lvtkIOCityGML-9.0
LIBS+=-lvtkIOCore-9.0
LIBS+=-lvtkIOEnSight-9.0
LIBS+=-lvtkIOExodus-9.0
LIBS+=-lvtkIOExport-9.0
LIBS+=-lvtkIOExportGL2PS-9.0
LIBS+=-lvtkIOExportPDF-9.0
LIBS+=-lvtkIOGeometry-9.0
LIBS+=-lvtkIOImage-9.0
LIBS+=-lvtkIOImport-9.0
LIBS+=-lvtkIOInfovis-9.0
LIBS+=-lvtkIOLegacy-9.0
LIBS+=-lvtkIOLSDyna-9.0
LIBS+=-lvtkIOMINC-9.0
LIBS+=-lvtkIOMotionFX-9.0
LIBS+=-lvtkIOMovie-9.0
LIBS+=-lvtkIONetCDF-9.0
LIBS+=-lvtkIOOggTheora-9.0
LIBS+=-lvtkIOParallel-9.0
LIBS+=-lvtkIOParallelXML-9.0
LIBS+=-lvtkIOPLY-9.0
LIBS+=-lvtkIOSegY-9.0
LIBS+=-lvtkIOSQL-9.0
LIBS+=-lvtkIOTecplotTable-9.0
LIBS+=-lvtkIOVeraOut-9.0
LIBS+=-lvtkIOVideo-9.0
LIBS+=-lvtkIOXML-9.0
LIBS+=-lvtkIOXMLParser-9.0
LIBS+=-lvtkjpeg-9.0
LIBS+=-lvtkjsoncpp-9.0
LIBS+=-lvtklibharu-9.0
LIBS+=-lvtklibproj-9.0
LIBS+=-lvtklibxml2-9.0
LIBS+=-lvtkloguru-9.0
LIBS+=-lvtklz4-9.0
LIBS+=-lvtklzma-9.0
LIBS+=-lvtkmetaio-9.0
LIBS+=-lvtknetcdf-9.0
LIBS+=-lvtkogg-9.0
LIBS+=-lvtkParallelCore-9.0
LIBS+=-lvtkParallelDIY-9.0
LIBS+=-lvtkpng-9.0
LIBS+=-lvtkpugixml-9.0
LIBS+=-lvtkRenderingAnnotation-9.0
LIBS+=-lvtkRenderingContext2D-9.0
LIBS+=-lvtkRenderingContextOpenGL2-9.0
LIBS+=-lvtkRenderingCore-9.0
LIBS+=-lvtkRenderingFreeType-9.0
LIBS+=-lvtkRenderingGL2PSOpenGL2-9.0
LIBS+=-lvtkRenderingImage-9.0
LIBS+=-lvtkRenderingLabel-9.0
LIBS+=-lvtkRenderingLOD-9.0
LIBS+=-lvtkRenderingOpenGL2-9.0
LIBS+=-lvtkRenderingQt-9.0
LIBS+=-lvtkRenderingSceneGraph-9.0
LIBS+=-lvtkRenderingUI-9.0
LIBS+=-lvtkRenderingVolume-9.0
LIBS+=-lvtkRenderingVolumeOpenGL2-9.0
LIBS+=-lvtkRenderingvtkJS-9.0
LIBS+=-lvtksqlite-9.0
LIBS+=-lvtksys-9.0
LIBS+=-lvtkTestingRendering-9.0
LIBS+=-lvtktheora-9.0
LIBS+=-lvtktiff-9.0
LIBS+=-lvtkverdict-9.0
LIBS+=-lvtkViewsContext2D-9.0
LIBS+=-lvtkViewsCore-9.0
LIBS+=-lvtkViewsInfovis-9.0
LIBS+=-lvtkViewsQt-9.0
LIBS+=-lvtkWrappingTools-9.0
LIBS+=-lvtkzlib-9.0
