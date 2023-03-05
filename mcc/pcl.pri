INCLUDEPATH += D:\PCL\PCL1.12.1\include\pcl-1.12


INCLUDEPATH += D:\PCL\PCL1.12.1\include\pcl-1.12\pcl


INCLUDEPATH += D:\PCL\PCL1.12.1\3rdParty\Boost\include\boost-1_78


INCLUDEPATH += D:\PCL\PCL1.12.1\3rdParty\Boost\include\boost-1_78\boost


INCLUDEPATH += D:\PCL\PCL1.12.1\3rdParty\Eigen\eigen3


INCLUDEPATH += D:\PCL\PCL1.12.1\3rdParty\FLANN\include


INCLUDEPATH += D:\PCL\PCL1.12.1\3rdParty\FLANN\include\flann


INCLUDEPATH += D:\PCL\PCL1.12.1\3rdParty\OpenNI2\Include


INCLUDEPATH += D:\PCL\PCL1.12.1\3rdParty\Qhull\include


INCLUDEPATH += D:\PCL\PCL1.12.1\3rdParty\VTK\include\vtk-9.1


INCLUDEPATH += D:\VTK\VTK-9.1.0.rc2-install-release\include\vtk-9.1


INCLUDEPATH += D:\VTK\VTK-9.1.0.rc2-install-debug\include\vtk-9.1


INCLUDEPATH += D:\PCL\PCL1.12.1\bin


CONFIG(debug,debug|release){
LIBS += -LD:\PCL\PCL1.12.1\lib\
-lpcl_commond\
-lpcl_featuresd\
-lpcl_filtersd\
-lpcl_iod\
-lpcl_io_plyd\
-lpcl_kdtreed\
-lpcl_keypointsd\
-lpcl_mld\
-lpcl_octreed\
-lpcl_outofcored\
-lpcl_peopled\
-lpcl_recognitiond\
-lpcl_registrationd\
-lpcl_sample_consensusd\
-lpcl_searchd\
-lpcl_segmentationd\
-lpcl_stereod\
-lpcl_surfaced\
-lpcl_trackingd\
-lpcl_visualizationd



LIBS += -LD:\PCL\PCL1.12.1\3rdParty\Boost\lib\
-llibboost_atomic-vc142-mt-gd-x64-1_78\
-llibboost_bzip2-vc142-mt-gd-x64-1_78\
-llibboost_chrono-vc142-mt-gd-x64-1_78\
-llibboost_container-vc142-mt-gd-x64-1_78\
-llibboost_context-vc142-mt-gd-x64-1_78\
-llibboost_contract-vc142-mt-gd-x64-1_78\
-llibboost_coroutine-vc142-mt-gd-x64-1_78\
-llibboost_date_time-vc142-mt-gd-x64-1_78\
-llibboost_exception-vc142-mt-gd-x64-1_78\
#-llibboost_fiber-vc142-mt-gd-x64-1_78\
-llibboost_filesystem-vc142-mt-gd-x64-1_78\
-llibboost_graph-vc142-mt-gd-x64-1_78\
-llibboost_graph_parallel-vc142-mt-gd-x64-1_78\
-llibboost_iostreams-vc142-mt-gd-x64-1_78\
-llibboost_locale-vc142-mt-gd-x64-1_78\
-llibboost_log-vc142-mt-gd-x64-1_78\
-llibboost_log_setup-vc142-mt-gd-x64-1_78\
-llibboost_math_c99-vc142-mt-gd-x64-1_78\
-llibboost_math_c99f-vc142-mt-gd-x64-1_78\
-llibboost_math_c99l-vc142-mt-gd-x64-1_78\
-llibboost_math_tr1-vc142-mt-gd-x64-1_78\
-llibboost_math_tr1f-vc142-mt-gd-x64-1_78\
-llibboost_math_tr1l-vc142-mt-gd-x64-1_78\
-llibboost_mpi-vc142-mt-gd-x64-1_78\
-llibboost_nowide-vc142-mt-gd-x64-1_78\
-llibboost_numpy38-vc142-mt-gd-x64-1_78\
-llibboost_prg_exec_monitor-vc142-mt-gd-x64-1_78\
-llibboost_program_options-vc142-mt-gd-x64-1_78\
-llibboost_python38-vc142-mt-gd-x64-1_78\
-llibboost_random-vc142-mt-gd-x64-1_78\
-llibboost_regex-vc142-mt-gd-x64-1_78\
-llibboost_serialization-vc142-mt-gd-x64-1_78\
#-llibboost_stacktrace_noop-vc142-mt-gd-x64-1_78\
#-llibboost_stacktrace_windbg-vc142-mt-gd-x64-1_78\
#-llibboost_stacktrace_windbg_cached-vc142-mt-gd-x64-1_78\
-llibboost_system-vc142-mt-gd-x64-1_78\
-llibboost_test_exec_monitor-vc142-mt-gd-x64-1_78\
-llibboost_thread-vc142-mt-gd-x64-1_78\
-llibboost_timer-vc142-mt-gd-x64-1_78\
-llibboost_type_erasure-vc142-mt-gd-x64-1_78\
-llibboost_unit_test_framework-vc142-mt-gd-x64-1_78\
-llibboost_wave-vc142-mt-gd-x64-1_78\
-llibboost_wserialization-vc142-mt-gd-x64-1_78\
-llibboost_zlib-vc142-mt-gd-x64-1_78


LIBS += -LD:\PCL\PCL1.12.1\3rdParty\FLANN\lib\
        -lflann-gd\
        -lflann_cpp-gd\
        -lflann_cpp_s-gd\
        -lflann_s-gd

LIBS += -LD:\PCL\PCL1.12.1\3rdParty\OpenNI2\Lib\
        -lOpenNI2

LIBS += -LD:\PCL\PCL1.12.1\3rdParty\Qhull\lib\
#        -lqhull_d\
        -lqhullcpp_d\
        -lqhullstatic_d\
        -lqhullstatic_rd\
#        -lqhull_p_d\
        -lqhull_rd

LIBS += -LD:\VTK\VTK-9.1.0.rc2-install-debug\lib\
-lvtkcgns-9.1d\
-lvtkChartsCore-9.1d\
-lvtkCommonColor-9.1d\
-lvtkCommonComputationalGeometry-9.1d\
-lvtkCommonCore-9.1d\
-lvtkCommonDataModel-9.1d\
-lvtkCommonExecutionModel-9.1d\
-lvtkCommonMath-9.1d\
-lvtkCommonMisc-9.1d\
-lvtkCommonSystem-9.1d\
-lvtkCommonTransforms-9.1d\
-lvtkDICOMParser-9.1d\
-lvtkDomainsChemistry-9.1d\
-lvtkDomainsChemistryOpenGL2-9.1d\
-lvtkdoubleconversion-9.1d\
-lvtkexodusII-9.1d\
-lvtkexpat-9.1d\
-lvtkFiltersAMR-9.1d\
-lvtkFiltersCore-9.1d\
-lvtkFiltersExtraction-9.1d\
-lvtkFiltersFlowPaths-9.1d\
-lvtkFiltersGeneral-9.1d\
-lvtkFiltersGeneric-9.1d\
-lvtkFiltersGeometry-9.1d\
-lvtkFiltersHybrid-9.1d\
-lvtkFiltersHyperTree-9.1d\
-lvtkFiltersImaging-9.1d\
-lvtkFiltersModeling-9.1d\
-lvtkFiltersParallel-9.1d\
-lvtkFiltersParallelImaging-9.1d\
-lvtkFiltersPoints-9.1d\
-lvtkFiltersProgrammable-9.1d\
-lvtkFiltersSelection-9.1d\
-lvtkFiltersSMP-9.1d\
-lvtkFiltersSources-9.1d\
-lvtkFiltersStatistics-9.1d\
-lvtkFiltersTexture-9.1d\
-lvtkFiltersTopology-9.1d\
-lvtkFiltersVerdict-9.1d\
-lvtkfmt-9.1d\
-lvtkfreetype-9.1d\
-lvtkGeovisCore-9.1d\
-lvtkgl2ps-9.1d\
-lvtkglew-9.1d\
-lvtkGUISupportQt-9.1d\
-lvtkGUISupportQtQuick-9.1d\
-lvtkGUISupportQtSQL-9.1d\
-lvtkhdf5-9.1d\
-lvtkhdf5_hl-9.1d\
-lvtkImagingColor-9.1d\
-lvtkImagingCore-9.1d\
-lvtkImagingFourier-9.1d\
-lvtkImagingGeneral-9.1d\
-lvtkImagingHybrid-9.1d\
-lvtkImagingMath-9.1d\
-lvtkImagingMorphological-9.1d\
-lvtkImagingSources-9.1d\
-lvtkImagingStatistics-9.1d\
-lvtkImagingStencil-9.1d\
-lvtkInfovisCore-9.1d\
-lvtkInfovisLayout-9.1d\
-lvtkInteractionImage-9.1d\
-lvtkInteractionStyle-9.1d\
-lvtkInteractionWidgets-9.1d\
-lvtkIOAMR-9.1d\
-lvtkIOAsynchronous-9.1d\
-lvtkIOCGNSReader-9.1d\
-lvtkIOChemistry-9.1d\
-lvtkIOCityGML-9.1d\
-lvtkIOCONVERGECFD-9.1d\
-lvtkIOCore-9.1d\
-lvtkIOEnSight-9.1d\
-lvtkIOExodus-9.1d\
-lvtkIOExport-9.1d\
-lvtkIOExportGL2PS-9.1d\
-lvtkIOExportPDF-9.1d\
-lvtkIOGeometry-9.1d\
-lvtkIOHDF-9.1d\
-lvtkIOImage-9.1d\
-lvtkIOImport-9.1d\
-lvtkIOInfovis-9.1d\
-lvtkIOIOSS-9.1d\
-lvtkIOLegacy-9.1d\
-lvtkIOLSDyna-9.1d\
-lvtkIOMINC-9.1d\
-lvtkIOMotionFX-9.1d\
-lvtkIOMovie-9.1d\
-lvtkIONetCDF-9.1d\
-lvtkIOOggTheora-9.1d\
-lvtkIOParallel-9.1d\
-lvtkIOParallelXML-9.1d\
-lvtkIOPLY-9.1d\
-lvtkIOSegY-9.1d\
-lvtkIOSQL-9.1d\
-lvtkioss-9.1d\
-lvtkIOTecplotTable-9.1d\
-lvtkIOVeraOut-9.1d\
-lvtkIOVideo-9.1d\
-lvtkIOXML-9.1d\
-lvtkIOXMLParser-9.1d\
-lvtkjpeg-9.1d\
-lvtkjsoncpp-9.1d\
-lvtkkissfft-9.1d\
-lvtklibharu-9.1d\
-lvtklibproj-9.1d\
-lvtklibxml2-9.1d\
-lvtkloguru-9.1d\
-lvtklz4-9.1d\
-lvtklzma-9.1d\
-lvtkmetaio-9.1d\
-lvtknetcdf-9.1d\
-lvtkogg-9.1d\
-lvtkParallelCore-9.1d\
-lvtkParallelDIY-9.1d\
-lvtkpng-9.1d\
-lvtkpugixml-9.1d\
-lvtkRenderingAnnotation-9.1d\
-lvtkRenderingContext2D-9.1d\
-lvtkRenderingContextOpenGL2-9.1d\
-lvtkRenderingCore-9.1d\
-lvtkRenderingFreeType-9.1d\
-lvtkRenderingGL2PSOpenGL2-9.1d\
-lvtkRenderingImage-9.1d\
-lvtkRenderingLabel-9.1d\
-lvtkRenderingLOD-9.1d\
-lvtkRenderingOpenGL2-9.1d\
-lvtkRenderingQt-9.1d\
-lvtkRenderingSceneGraph-9.1d\
-lvtkRenderingUI-9.1d\
-lvtkRenderingVolume-9.1d\
-lvtkRenderingVolumeOpenGL2-9.1d\
-lvtkRenderingVtkJS-9.1d\
-lvtksqlite-9.1d\
-lvtksys-9.1d\
-lvtkTestingRendering-9.1d\
-lvtktheora-9.1d\
-lvtktiff-9.1d\
-lvtkverdict-9.1d\
-lvtkViewsContext2D-9.1d\
-lvtkViewsCore-9.1d\
-lvtkViewsInfovis-9.1d\
-lvtkViewsQt-9.1d\
-lvtkWrappingTools-9.1d\
-lvtkzlib-9.1d

        }
else {
LIBS += -LD:\PCL\PCL1.12.1\lib\
        -lpcl_common\
        -lpcl_features\
        -lpcl_filters\
        -lpcl_io\
        -lpcl_io_ply\
        -lpcl_kdtree\
        -lpcl_keypoints\
        -lpcl_ml\
        -lpcl_octree\
        -lpcl_outofcore\
        -lpcl_people\
        -lpcl_recognition\
        -lpcl_registration\
        -lpcl_sample_consensus\
        -lpcl_search\
        -lpcl_segmentation\
        -lpcl_stereo\
        -lpcl_surface\
        -lpcl_tracking\
        -lpcl_visualization

LIBS += -LD:\PCL\PCL1.12.1\3rdParty\Boost\lib\
        -llibboost_atomic-vc142-mt-gd-x64-1_78\
        -llibboost_bzip2-vc142-mt-gd-x64-1_78\
        -llibboost_chrono-vc142-mt-gd-x64-1_78\
        -llibboost_container-vc142-mt-gd-x64-1_78\
        -llibboost_context-vc142-mt-gd-x64-1_78\
        -llibboost_coroutine-vc142-mt-gd-x64-1_78\
        -llibboost_date_time-vc142-mt-gd-x64-1_78\
        -llibboost_exception-vc142-mt-gd-x64-1_78\
        -llibboost_filesystem-vc142-mt-gd-x64-1_78\
        -llibboost_graph-vc142-mt-gd-x64-1_78\
        -llibboost_graph_parallel-vc142-mt-gd-x64-1_78\
        -llibboost_iostreams-vc142-mt-gd-x64-1_78\
        -llibboost_locale-vc142-mt-gd-x64-1_78\
        -llibboost_log-vc142-mt-gd-x64-1_78\
        -llibboost_log_setup-vc142-mt-gd-x64-1_78\
        -llibboost_math_c99-vc142-mt-gd-x64-1_78\
        -llibboost_math_c99f-vc142-mt-gd-x64-1_78\
        -llibboost_math_c99l-vc142-mt-gd-x64-1_78\
        -llibboost_math_tr1-vc142-mt-gd-x64-1_78\
        -llibboost_math_tr1f-vc142-mt-gd-x64-1_78\
        -llibboost_math_tr1l-vc142-mt-gd-x64-1_78\
        -llibboost_mpi-vc142-mt-gd-x64-1_78\
        -llibboost_numpy38-vc142-mt-gd-x64-1_78\
        -llibboost_prg_exec_monitor-vc142-mt-gd-x64-1_78\
        -llibboost_program_options-vc142-mt-gd-x64-1_78\
        -llibboost_python38-vc142-mt-gd-x64-1_78\
        -llibboost_python38-vc142-mt-gd-x64-1_78\
        -llibboost_random-vc142-mt-gd-x64-1_78\
        -llibboost_regex-vc142-mt-gd-x64-1_78\
        -llibboost_serialization-vc142-mt-gd-x64-1_78\
        -llibboost_system-vc142-mt-gd-x64-1_78\
        -llibboost_test_exec_monitor-vc142-mt-gd-x64-1_78\
        -llibboost_thread-vc142-mt-gd-x64-1_78\
        -llibboost_timer-vc142-mt-gd-x64-1_78\
        -llibboost_type_erasure-vc142-mt-gd-x64-1_78\
        -llibboost_unit_test_framework-vc142-mt-gd-x64-1_78\
        -llibboost_wave-vc142-mt-gd-x64-1_78\
        -llibboost_wserialization-vc142-mt-gd-x64-1_78\
        -llibboost_zlib-vc142-mt-gd-x64-1_78

LIBS += -LD:\PCL\PCL1.12.1\3rdParty\FLANN\lib\
        -lflann\
        -lflann_cpp\
        -lflann_cpp_s\
        -lflann_s

LIBS += -LD:\PCL\PCL1.12.1\3rdParty\OpenNI2\Lib\
        -lOpenNI2

LIBS += -LD:\PCL\PCL1.12.1\3rdParty\Qhull\lib\
        -lqhull_rd\
        -lqhullcpp_d\
        -lqhullstatic_d\
        -lqhullstatic_rd

LIBS += -LD:\VTK\VTK-9.1.0.rc2-install-release\lib\
-lvtkcgns-9.1\
-lvtkChartsCore-9.1\
-lvtkCommonColor-9.1\
-lvtkCommonComputationalGeometry-9.1\
-lvtkCommonCore-9.1\
-lvtkCommonDataModel-9.1\
-lvtkCommonExecutionModel-9.1\
-lvtkCommonMath-9.1\
-lvtkCommonMisc-9.1\
-lvtkCommonSystem-9.1\
-lvtkCommonTransforms-9.1\
-lvtkDICOMParser-9.1\
-lvtkDomainsChemistry-9.1\
-lvtkDomainsChemistryOpenGL2-9.1\
-lvtkdoubleconversion-9.1\
-lvtkexodusII-9.1\
-lvtkexpat-9.1\
-lvtkFiltersAMR-9.1\
-lvtkFiltersCore-9.1\
-lvtkFiltersExtraction-9.1\
-lvtkFiltersFlowPaths-9.1\
-lvtkFiltersGeneral-9.1\
-lvtkFiltersGeneric-9.1\
-lvtkFiltersGeometry-9.1\
-lvtkFiltersHybrid-9.1\
-lvtkFiltersHyperTree-9.1\
-lvtkFiltersImaging-9.1\
-lvtkFiltersModeling-9.1\
-lvtkFiltersParallel-9.1\
-lvtkFiltersParallelImaging-9.1\
-lvtkFiltersPoints-9.1\
-lvtkFiltersProgrammable-9.1\
-lvtkFiltersSelection-9.1\
-lvtkFiltersSMP-9.1\
-lvtkFiltersSources-9.1\
-lvtkFiltersStatistics-9.1\
-lvtkFiltersTexture-9.1\
-lvtkFiltersTopology-9.1\
-lvtkFiltersVerdict-9.1\
-lvtkfmt-9.1\
-lvtkfreetype-9.1\
-lvtkGeovisCore-9.1\
-lvtkgl2ps-9.1\
-lvtkglew-9.1\
-lvtkGUISupportQt-9.1\
-lvtkGUISupportQtQuick-9.1\
-lvtkGUISupportQtSQL-9.1\
-lvtkhdf5-9.1\
-lvtkhdf5_hl-9.1\
-lvtkImagingColor-9.1\
-lvtkImagingCore-9.1\
-lvtkImagingFourier-9.1\
-lvtkImagingGeneral-9.1\
-lvtkImagingHybrid-9.1\
-lvtkImagingMath-9.1\
-lvtkImagingMorphological-9.1\
-lvtkImagingSources-9.1\
-lvtkImagingStatistics-9.1\
-lvtkImagingStencil-9.1\
-lvtkInfovisCore-9.1\
-lvtkInfovisLayout-9.1\
-lvtkInteractionImage-9.1\
-lvtkInteractionStyle-9.1\
-lvtkInteractionWidgets-9.1\
-lvtkIOAMR-9.1\
-lvtkIOAsynchronous-9.1\
-lvtkIOCGNSReader-9.1\
-lvtkIOChemistry-9.1\
-lvtkIOCityGML-9.1\
-lvtkIOCONVERGECFD-9.1\
-lvtkIOCore-9.1\
-lvtkIOEnSight-9.1\
-lvtkIOExodus-9.1\
-lvtkIOExport-9.1\
-lvtkIOExportGL2PS-9.1\
-lvtkIOExportPDF-9.1\
-lvtkIOGeometry-9.1\
-lvtkIOHDF-9.1\
-lvtkIOImage-9.1\
-lvtkIOImport-9.1\
-lvtkIOInfovis-9.1\
-lvtkIOIOSS-9.1\
-lvtkIOLegacy-9.1\
-lvtkIOLSDyna-9.1\
-lvtkIOMINC-9.1\
-lvtkIOMotionFX-9.1\
-lvtkIOMovie-9.1\
-lvtkIONetCDF-9.1\
-lvtkIOOggTheora-9.1\
-lvtkIOParallel-9.1\
-lvtkIOParallelXML-9.1\
-lvtkIOPLY-9.1\
-lvtkIOSegY-9.1\
-lvtkIOSQL-9.1\
-lvtkioss-9.1\
-lvtkIOTecplotTable-9.1\
-lvtkIOVeraOut-9.1\
-lvtkIOVideo-9.1\
-lvtkIOXML-9.1\
-lvtkIOXMLParser-9.1\
-lvtkjpeg-9.1\
-lvtkjsoncpp-9.1\
-lvtkkissfft-9.1\
-lvtklibharu-9.1\
-lvtklibproj-9.1\
-lvtklibxml2-9.1\
-lvtkloguru-9.1\
-lvtklz4-9.1\
-lvtklzma-9.1\
-lvtkmetaio-9.1\
-lvtknetcdf-9.1\
-lvtkogg-9.1\
-lvtkParallelCore-9.1\
-lvtkParallelDIY-9.1\
-lvtkpng-9.1\
-lvtkpugixml-9.1\
-lvtkRenderingAnnotation-9.1\
-lvtkRenderingContext2D-9.1\
-lvtkRenderingContextOpenGL2-9.1\
-lvtkRenderingCore-9.1\
-lvtkRenderingFreeType-9.1\
-lvtkRenderingGL2PSOpenGL2-9.1\
-lvtkRenderingImage-9.1\
-lvtkRenderingLabel-9.1\
-lvtkRenderingLOD-9.1\
-lvtkRenderingOpenGL2-9.1\
-lvtkRenderingQt-9.1\
-lvtkRenderingSceneGraph-9.1\
-lvtkRenderingUI-9.1\
-lvtkRenderingVolume-9.1\
-lvtkRenderingVolumeOpenGL2-9.1\
-lvtkRenderingVtkJS-9.1\
-lvtksqlite-9.1\
-lvtksys-9.1\
-lvtkTestingRendering-9.1\
-lvtktheora-9.1\
-lvtktiff-9.1\
-lvtkverdict-9.1\
-lvtkViewsContext2D-9.1\
-lvtkViewsCore-9.1\
-lvtkViewsInfovis-9.1\
-lvtkViewsQt-9.1\
-lvtkWrappingTools-9.1\
-lvtkzlib-9.1
}

#CONFIG(debug,debug|release){

#LIBS += -LD:\PCL\PCL1.12.1\lib\
#        -lpcl_common\
#        -lpcl_features\
#        -lpcl_filters\
#        -lpcl_io\
#        -lpcl_io_ply\
#        -lpcl_kdtree\
#        -lpcl_keypoints\
#        -lpcl_ml\
#        -lpcl_octree\
#        -lpcl_outofcore\
#        -lpcl_people\
#        -lpcl_recognition\
#        -lpcl_registration\
#        -lpcl_sample_consensus\
#        -lpcl_search\
#        -lpcl_segmentation\
#        -lpcl_stereo\
#        -lpcl_surface\
#        -lpcl_tracking\
#        -lpcl_visualization

#LIBS += -LD:\PCL\PCL1.12.1\3rdParty\Boost\lib\
#        -llibboost_atomic-vc142-mt-gd-x64-1_78\
#        -llibboost_bzip2-vc142-mt-gd-x64-1_78\
#        -llibboost_chrono-vc142-mt-gd-x64-1_78\
#        -llibboost_container-vc142-mt-gd-x64-1_78\
#        -llibboost_context-vc142-mt-gd-x64-1_78\
#        -llibboost_coroutine-vc142-mt-gd-x64-1_78\
#        -llibboost_date_time-vc142-mt-gd-x64-1_78\
#        -llibboost_exception-vc142-mt-gd-x64-1_78\
#        -llibboost_filesystem-vc142-mt-gd-x64-1_78\
#        -llibboost_graph-vc142-mt-gd-x64-1_78\
#        -llibboost_graph_parallel-vc142-mt-gd-x64-1_78\
#        -llibboost_iostreams-vc142-mt-gd-x64-1_78\
#        -llibboost_locale-vc142-mt-gd-x64-1_78\
#        -llibboost_log-vc142-mt-gd-x64-1_78\
#        -llibboost_log_setup-vc142-mt-gd-x64-1_78\
#        -llibboost_math_c99-vc142-mt-gd-x64-1_78\
#        -llibboost_math_c99f-vc142-mt-gd-x64-1_78\
#        -llibboost_math_c99l-vc142-mt-gd-x64-1_78\
#        -llibboost_math_tr1-vc142-mt-gd-x64-1_78\
#        -llibboost_math_tr1f-vc142-mt-gd-x64-1_78\
#        -llibboost_math_tr1l-vc142-mt-gd-x64-1_78\
#        -llibboost_mpi-vc142-mt-gd-x64-1_78\
#        -llibboost_numpy38-vc142-mt-gd-x64-1_78\
#        -llibboost_prg_exec_monitor-vc142-mt-gd-x64-1_78\
#        -llibboost_program_options-vc142-mt-gd-x64-1_78\
#        -llibboost_python38-vc142-mt-gd-x64-1_78\
#        -llibboost_python38-vc142-mt-gd-x64-1_78\
#        -llibboost_random-vc142-mt-gd-x64-1_78\
#        -llibboost_regex-vc142-mt-gd-x64-1_78\
#        -llibboost_serialization-vc142-mt-gd-x64-1_78\
#        -llibboost_system-vc142-mt-gd-x64-1_78\
#        -llibboost_test_exec_monitor-vc142-mt-gd-x64-1_78\
#        -llibboost_thread-vc142-mt-gd-x64-1_78\
#        -llibboost_timer-vc142-mt-gd-x64-1_78\
#        -llibboost_type_erasure-vc142-mt-gd-x64-1_78\
#        -llibboost_unit_test_framework-vc142-mt-gd-x64-1_78\
#        -llibboost_wave-vc142-mt-gd-x64-1_78\
#        -llibboost_wserialization-vc142-mt-gd-x64-1_78\
#        -llibboost_zlib-vc142-mt-gd-x64-1_78

#LIBS += -LD:\PCL\PCL1.12.1\3rdParty\FLANN\lib\
#        -lflann-gd\
#        -lflann_cpp-gd\
#        -lflann_cpp_s-gd\
#        -lflann_s-gd

#LIBS += -LD:\PCL\PCL1.12.1\3rdParty\OpenNI2\Lib\
#        -lOpenNI2

#LIBS += -LD:\PCL\PCL1.12.1\3rdParty\Qhull\lib\
#        -lqhull_rd\
#        -lqhullcpp_d\
#        -lqhullstatic_d\
#        -lqhullstatic_rd

#LIBS += -LC:\Qt\Qt5.12.12\5.12.12\msvc2017_64\lib\
#                -lQt5OpenGL\
#                -lQt5Core\
#                -lQt5Gui\
#                -lQt5Widgets

##LIBS += -LD:\VTK9.1.0\vtk_install\lib\
##                -lvtkRenderingOpenGL2-9.1\
##                -lvtkRenderingFreeType-9.1\
##                -lvtkInteractionStyle-9.1\
##                -lvtkInteractionStyle-9.1-gd\
##                -lvtkRenderingFreeType-9.1-gd\
##                -lvtkInteractionStyle-9.1-gd

#LIBS += -LD:\PCL\PCL1.12.1\3rdParty\VTK\lib\
#        vtkcgns-9.1d.lib
#        vtkChartsCore-9.1d.lib
#        vtkCommonColor-9.1d.lib
#        vtkCommonComputationalGeometry-9.1d.lib
#        vtkCommonCore-9.1d.lib
#        vtkCommonDataModel-9.1d.lib
#        vtkCommonExecutionModel-9.1d.lib
#        vtkCommonMath-9.1d.lib
#        vtkCommonMisc-9.1d.lib
#        vtkCommonSystem-9.1d.lib
#        vtkCommonTransforms-9.1d.lib
#        vtkDICOMParser-9.1d.lib
#        vtkDomainsChemistry-9.1d.lib
#        vtkDomainsChemistryOpenGL2-9.1d.lib
#        vtkdoubleconversion-9.1d.lib
#        vtkexodusII-9.1d.lib
#        vtkexpat-9.1d.lib
#        vtkFiltersAMR-9.1d.lib
#        vtkFiltersCore-9.1d.lib
#        vtkFiltersExtraction-9.1d.lib
#        vtkFiltersFlowPaths-9.1d.lib
#        vtkFiltersGeneral-9.1d.lib
#        vtkFiltersGeneric-9.1d.lib
#        vtkFiltersGeometry-9.1d.lib
#        vtkFiltersHybrid-9.1d.lib
#        vtkFiltersHyperTree-9.1d.lib
#        vtkFiltersImaging-9.1d.lib
#        vtkFiltersModeling-9.1d.lib
#        vtkFiltersParallel-9.1d.lib
#        vtkFiltersParallelImaging-9.1d.lib
#        vtkFiltersPoints-9.1d.lib
#        vtkFiltersProgrammable-9.1d.lib
#        vtkFiltersSelection-9.1d.lib
#        vtkFiltersSMP-9.1d.lib
#        vtkFiltersSources-9.1d.lib
#        vtkFiltersStatistics-9.1d.lib
#        vtkFiltersTexture-9.1d.lib
#        vtkFiltersTopology-9.1d.lib
#        vtkFiltersVerdict-9.1d.lib
#        vtkfmt-9.1d.lib
#        vtkfreetype-9.1d.lib
#        vtkGeovisCore-9.1d.lib
#        vtkgl2ps-9.1d.lib
#        vtkglew-9.1d.lib
#        vtkhdf5-9.1d.lib
#        vtkhdf5_hl-9.1d.lib
#        vtkImagingColor-9.1d.lib
#        vtkImagingCore-9.1d.lib
#        vtkImagingFourier-9.1d.lib
#        vtkImagingGeneral-9.1d.lib
#        vtkImagingHybrid-9.1d.lib
#        vtkImagingMath-9.1d.lib
#        vtkImagingMorphological-9.1d.lib
#        vtkImagingSources-9.1d.lib
#        vtkImagingStatistics-9.1d.lib
#        vtkImagingStencil-9.1d.lib
#        vtkInfovisCore-9.1d.lib
#        vtkInfovisLayout-9.1d.lib
#        vtkInteractionImage-9.1d.lib
#        vtkInteractionStyle-9.1d.lib
#        vtkInteractionWidgets-9.1d.lib
#        vtkIOAMR-9.1d.lib
#        vtkIOAsynchronous-9.1d.lib
#        vtkIOCGNSReader-9.1d.lib
#        vtkIOChemistry-9.1d.lib
#        vtkIOCityGML-9.1d.lib
#        vtkIOCONVERGECFD-9.1d.lib
#        vtkIOCore-9.1d.lib
#        vtkIOEnSight-9.1d.lib
#        vtkIOExodus-9.1d.lib
#        vtkIOExport-9.1d.lib
#        vtkIOExportGL2PS-9.1d.lib
#        vtkIOExportPDF-9.1d.lib
#        vtkIOGeometry-9.1d.lib
#        vtkIOHDF-9.1d.lib
#        vtkIOImage-9.1d.lib
#        vtkIOImport-9.1d.lib
#        vtkIOInfovis-9.1d.lib
#        vtkIOIOSS-9.1d.lib
#        vtkIOLegacy-9.1d.lib
#        vtkIOLSDyna-9.1d.lib
#        vtkIOMINC-9.1d.lib
#        vtkIOMotionFX-9.1d.lib
#        vtkIOMovie-9.1d.lib
#        vtkIONetCDF-9.1d.lib
#        vtkIOOggTheora-9.1d.lib
#        vtkIOParallel-9.1d.lib
#        vtkIOParallelXML-9.1d.lib
#        vtkIOPLY-9.1d.lib
#        vtkIOSegY-9.1d.lib
#        vtkIOSQL-9.1d.lib
#        vtkioss-9.1d.lib
#        vtkIOTecplotTable-9.1d.lib
#        vtkIOVeraOut-9.1d.lib
#        vtkIOVideo-9.1d.lib
#        vtkIOXML-9.1d.lib
#        vtkIOXMLParser-9.1d.lib
#        vtkjpeg-9.1d.lib
#        vtkjsoncpp-9.1d.lib
#        vtkkissfft-9.1d.lib
#        vtklibharu-9.1d.lib
#        vtklibproj-9.1d.lib
#        vtklibxml2-9.1d.lib
#        vtkloguru-9.1d.lib
#        vtklz4-9.1d.lib
#        vtklzma-9.1d.lib
#        vtkmetaio-9.1d.lib
#        vtknetcdf-9.1d.lib
#        vtkogg-9.1d.lib
#        vtkParallelCore-9.1d.lib
#        vtkParallelDIY-9.1d.lib
#        vtkpng-9.1d.lib
#        vtkpugixml-9.1d.lib
#        vtkRenderingAnnotation-9.1d.lib
#        vtkRenderingContext2D-9.1d.lib
#        vtkRenderingContextOpenGL2-9.1d.lib
#        vtkRenderingCore-9.1d.lib
#        vtkRenderingFreeType-9.1d.lib
#        vtkRenderingGL2PSOpenGL2-9.1d.lib
#        vtkRenderingImage-9.1d.lib
#        vtkRenderingLabel-9.1d.lib
#        vtkRenderingLOD-9.1d.lib
#        vtkRenderingOpenGL2-9.1d.lib
#        vtkRenderingSceneGraph-9.1d.lib
#        vtkRenderingUI-9.1d.lib
#        vtkRenderingVolume-9.1d.lib
#        vtkRenderingVolumeOpenGL2-9.1d.lib
#        vtkRenderingVtkJS-9.1d.lib
#        vtksqlite-9.1d.lib
#        vtksys-9.1d.lib
#        vtkTestingRendering-9.1d.lib
#        vtktheora-9.1d.lib
#        vtktiff-9.1d.lib
#        vtkverdict-9.1d.lib
#        vtkViewsContext2D-9.1d.lib
#        vtkViewsCore-9.1d.lib
#        vtkViewsInfovis-9.1d.lib
#        vtkWrappingTools-9.1d.lib
#        vtkzlib-9.1d.lib

#} else {

#LIBS += -LD:\PCL\PCL1.12.1\lib\
#                -lpcl_common\
#                -lpcl_features\
#                -lpcl_filters\
#                -lpcl_io_ply\
#                -lpcl_io\
#                -lpcl_kdtree\
#                -lpcl_keypoints\
#                -lpcl_ml\
#                -lpcl_octree\
#                -lpcl_outofcore\
#                -lpcl_people\
#                -lpcl_recognition\
#                -lpcl_registration\
#                -lpcl_sample_consensus\
#                -lpcl_search\
#                -lpcl_segmentation\
#                -lpcl_stereo\
#                -lpcl_surface\
#                -lpcl_tracking\
#                -lpcl_visualization

#        LIBS += -LD:\PCL\PCL1.12.1\3rdParty\Boost\lib\
#                -llibboost_atomic-vc142-mt-x64-1_78\
#                -llibboost_bzip2-vc142-mt-x64-1_78\
#                -llibboost_chrono-vc142-mt-x64-1_78\
#                -llibboost_container-vc142-mt-x64-1_78\
#                -llibboost_context-vc142-mt-x64-1_78\
#                -llibboost_coroutine-vc142-mt-x64-1_78\
#                -llibboost_date_time-vc142-mt-x64-1_78\
#                -llibboost_exception-vc142-mt-x64-1_78\
#                -llibboost_filesystem-vc142-mt-x64-1_78\
#                -llibboost_graph-vc142-mt-x64-1_78\
#                -llibboost_graph_parallel-vc142-mt-x64-1_78\
#                -llibboost_iostreams-vc142-mt-x64-1_78\
#                -llibboost_locale-vc142-mt-x64-1_78\
#                -llibboost_log-vc142-mt-x64-1_78\
#                -llibboost_log_setup-vc142-mt-x64-1_78\
#                -llibboost_math_c99-vc142-mt-x64-1_78\
#                -llibboost_math_c99f-vc142-mt-x64-1_78\
#                -llibboost_math_c99l-vc142-mt-x64-1_78\
#                -llibboost_math_tr1-vc142-mt-x64-1_78\
#                -llibboost_math_tr1f-vc142-mt-x64-1_78\
#                -llibboost_math_tr1l-vc142-mt-x64-1_78\
#                -llibboost_mpi-vc142-mt-x64-1_78\
#                -llibboost_nowide-vc142-mt-x64-1_78\
#                -llibboost_numpy38-vc142-mt-x64-1_78\
#                -llibboost_prg_exec_monitor-vc142-mt-x64-1_78\
#                -llibboost_program_options-vc142-mt-x64-1_78\
#                -llibboost_python38-vc142-mt-x64-1_78\
#                -llibboost_random-vc142-mt-x64-1_78\
#                -llibboost_regex-vc142-mt-x64-1_78\
#                -llibboost_serialization-vc142-mt-x64-1_78\
#                -llibboost_system-vc142-mt-x64-1_78\
#                -llibboost_test_exec_monitor-vc142-mt-x64-1_78\
#                -llibboost_thread-vc142-mt-x64-1_78\
#                -llibboost_timer-vc142-mt-x64-1_78\
#                -llibboost_type_erasure-vc142-mt-x64-1_78\
#                -llibboost_unit_test_framework-vc142-mt-x64-1_78\
#                -llibboost_wave-vc142-mt-x64-1_78\
#                -llibboost_wserialization-vc142-mt-x64-1_78\
#                -llibboost_zlib-vc142-mt-x64-1_78

#        LIBS += -LD:\PCL\PCL1.12.1\3rdParty\FLANN\lib\
#                -lflann\
#                -lflann_cpp\
#                -lflann_cpp_s\
#                -lflann_s

#        LIBS += -LD:\PCL\PCL1.12.1\3rdParty\OpenNI2\Lib\
#                -lOpenNI2

#        LIBS += -LD:\PCL\PCL1.12.1\3rdParty\Qhull\lib\
#                -lqhullcpp\
#                -lqhullstatic\
#                -lqhullstatic_r\
#                -lqhull_r

#        LIBS += -LD:\PCL\PCL1.12.1\3rdParty\VTK\lib\
#                -lvtkcgns-9.1\
#                -lvtkChartsCore-9.1\
#                -lvtkCommonColor-9.1\
#                -lvtkCommonComputationalGeometry-9.1\
#                -lvtkCommonCore-9.1\
#                -lvtkCommonDataModel-9.1\
#                -lvtkCommonExecutionModel-9.1\
#                -lvtkCommonMath-9.1\
#                -lvtkCommonMisc-9.1\
#                -lvtkCommonSystem-9.1\
#                -lvtkCommonTransforms-9.1\
#                -lvtkDICOMParser-9.1\
#                -lvtkDomainsChemistry-9.1\
#                -lvtkDomainsChemistryOpenGL2-9.1\
#                -lvtkdoubleconversion-9.1\
#                -lvtkexodusII-9.1\
#                -lvtkexpat-9.1\
#                -lvtkFiltersAMR-9.1\
#                -lvtkFiltersCore-9.1\
#                -lvtkFiltersExtraction-9.1\
#                -lvtkFiltersFlowPaths-9.1\
#                -lvtkFiltersGeneral-9.1\
#                -lvtkFiltersGeneric-9.1\
#                -lvtkFiltersGeometry-9.1\
#                -lvtkFiltersHybrid-9.1\
#                -lvtkFiltersHyperTree-9.1\
#                -lvtkFiltersImaging-9.1\
#                -lvtkFiltersModeling-9.1\
#                -lvtkFiltersParallel-9.1\
#                -lvtkFiltersParallelImaging-9.1\
#                -lvtkFiltersProgrammable-9.1\
#                -lvtkFiltersSelection-9.1\
#                -lvtkFiltersSMP-9.1\
#                -lvtkFiltersSources-9.1\
#                -lvtkFiltersStatistics-9.1\
#                -lvtkFiltersTexture-9.1\
#                -lvtkFiltersVerdict-9.1\
#                -lvtkfreetype-9.1\
#                -lvtkGeovisCore-9.1\
#                -lvtkglew-9.1\
#                -lvtkhdf5-9.1\
#                -lvtkhdf5_hl-9.1\
#                -lvtkImagingColor-9.1\
#                -lvtkImagingCore-9.1\
#                -lvtkImagingFourier-9.1\
#                -lvtkImagingGeneral-9.1\
#                -lvtkImagingHybrid-9.1\
#                -lvtkImagingMath-9.1\
#                -lvtkImagingMorphological-9.1\
#                -lvtkImagingSources-9.1\
#                -lvtkImagingStatistics-9.1\
#                -lvtkImagingStencil-9.1\
#                -lvtkInfovisCore-9.1\
#                -lvtkInfovisLayout-9.1\
#                -lvtkInteractionImage-9.1\
#                -lvtkInteractionStyle-9.1\
#                -lvtkInteractionWidgets-9.1\
#                -lvtkIOAMR-9.1\
#                -lvtkIOCore-9.1\
#                -lvtkIOEnSight-9.1\
#                -lvtkIOExodus-9.1\
#                -lvtkIOExport-9.1\
#                -lvtkIOGeometry-9.1\
#                -lvtkIOImage-9.1\
#                -lvtkIOImport-9.1\
#                -lvtkIOInfovis-9.1\
#                -lvtkIOLegacy-9.1\
#                -lvtkIOLSDyna-9.1\
#                -lvtkIOMINC-9.1\
#                -lvtkIOMovie-9.1\
#                -lvtkIONetCDF-9.1\
#                -lvtkIOParallel-9.1\
#                -lvtkIOParallelXML-9.1\
#                -lvtkIOPLY-9.1\
#                -lvtkIOSQL-9.1\
#                -lvtkIOVideo-9.1\
#                -lvtkIOXML-9.1\
#                -lvtkIOXMLParser-9.1\
#                -lvtkjpeg-9.1\
#                -lvtkjsoncpp-9.1\
#                -lvtklibxml2-9.1\
#                -lvtkmetaio-9.1\
#                -lvtkNetCDF-9.1\
#                -lvtkogg-9.1\
#                -lvtkParallelCore-9.1\
#                -lvtkParallelDIY-9.1\
#                -lvtkpng-9.1\
#                -lvtkpugixml-9.1\
#                -lvtkRenderingAnnotation-9.1\
#                -lvtkRenderingContext2D-9.1\
#                -lvtkRenderingContextOpenGL2-9.1\
#                -lvtkRenderingCore-9.1\
#                -lvtkRenderingFreeType-9.1\
#                -lvtkRenderingImage-9.1\
#                -lvtkRenderingLabel-9.1\
#                -lvtkRenderingLOD-9.1\
#                -lvtkRenderingOpenGL2-9.1\
#                -lvtkRenderingSceneGraph-9.1\
#                -lvtkRenderingUI-9.1\
#                -lvtkRenderingVolume-9.1\
#                -lvtkRenderingVolumeOpenGL2-9.1\
#                -lvtksqlite-9.1\
#                -lvtksys-9.1\
#                -lvtktiff-9.1\
#                -lvtkverdict-9.1\
#                -lvtkViewsContext2D-9.1\
#                -lvtkViewsCore-9.1\
#                -lvtkViewsInfovis-9.1\
#                -lvtkWrappingTools-9.1\
#                -lvtkzlib-9.1
#        }
