from qgis.gui import *
from qgis.core import *
#from qgis.PyQt.QtCore import Qt
from PyQt5.QtCore import *
from qgis.PyQt.QtWidgets import QAction, QMainWindow
import sys
import os
import pandas as pd
import datetime
#sys.path=['']+sys.path
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor



#path_to_folder="layers_custom/"

#lidar_layer=QgsPointCloudLayer("/home/pjourdan/qt_ws/layers_custom/PTS_LAMB93_IGN69_0925_6326.las","lidar", "pdal")
# zone_layer=QgsVectorLayer("/home/pjourdan/qt_ws/layers_custom/testzone.shp","monitoringZone", "ogr")
# zone_layer.loadNamedStyle('/home/pjourdan/qt_ws/style/zoneProbaGradua.qml')
#/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/


# if not lidar_layer.isValid():
    # print("Cloud Layer failed to load!")
# else:
    # QgsProject.instance().addMapLayer(lidar_layer)


def add_layer_toproject(layer,namety):
    if not layer.isValid():
        print(f"{namety} Layer failed to load!")
        #node.get_logger().info("Not ok load")
    else:
        QgsProject.instance().addMapLayer(layer)
        #node.get_logger().info("ok load")


def gen_lidarlayer():
    lidar_layer=QgsPointCloudLayer("/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/layers_custom/PTS_LAMB93_IGN69_0925_6326.las","lidar", "pdal")
    return lidar_layer

def gen_zonelayer():

    zone_layer=QgsVectorLayer("/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/layers_custom/testzone.shp","monitoringZone", "ogr")
    zone_layer.loadNamedStyle('/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/style/zoneProbaGradua.qml')
    features=zone_layer.getFeatures()
    layer_provider=zone_layer.dataProvider()
    zone_layer.startEditing()
    snew={'w0':1,'w1':0,'w2':0,'w3':1}
    index_field_state=zone_layer.fields().indexFromName('state')
    for f in features:
        valuestate=snew[f['name']]
        print(valuestate)
        layer_provider.changeAttributeValues({f.id():{index_field_state:valuestate}})
    zone_layer.commitChanges()
    return zone_layer



def gen_dronelayer():
    drone_layers = QgsVectorLayer("Point?crs=EPSG:4326", "temporary_points", "memory")
    pr = drone_layers.dataProvider()
    #drone_layers.startEditing()
    #print (QDateTime(2023, 5, 9, 17, 18, 58, 940))

    pr.addAttributes( [ QgsField("drone", QVariant.String),
                    QgsField("X",  QVariant.Int),
                    QgsField("Y",  QVariant.Int),
                   QgsField("date",QVariant.DateTime) ] ) 
    data_sim=pd.read_csv("/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/data/testdataPosi.csv")
    select_d1=data_sim.loc[data_sim['drone'] == 'd1']
    for index,row in select_d1.iterrows():
        formatted = datetime.datetime.strptime(row["datetime"],'%Y-%m-%d %H:%M:%S.%f')
        newdatebis=QDateTime(formatted.year, formatted.month, formatted.day, formatted.hour, formatted.minute, formatted.second)
        fet = QgsFeature()
        fet.setAttributes([row['drone'],row['x'],row['y'],newdatebis])
        fet.setGeometry( QgsGeometry.fromPointXY(QgsPointXY(row['x'],row['y'])) )
        pr.addFeatures( [ fet ] )               
    drone_layers.commitChanges()
    tempo_vl=drone_layers.temporalProperties()
    tempo_vl.setIsActive(True)
    tempo_vl.setStartField("date")
    tempo_vl.setMode(Qgis.VectorTemporalMode(1))
    return drone_layers
 

def gen_pathlayer():
    path_layers = QgsVectorLayer("LineString?crs=EPSG:4326", "temporary_lines", "memory")
    path_layers.loadNamedStyle('/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/style/arrowline.qml')
    pr = path_layers.dataProvider()
    path_layers.startEditing()
    pr.addAttributes( [ QgsField("drone", QVariant.String),
                    QgsField("X1",  QVariant.Int),
                    QgsField("Y1",  QVariant.Int),
                    QgsField("date1",QVariant.DateTime),
                    QgsField("X2",  QVariant.Int),
                    QgsField("Y2",  QVariant.Int),
                   QgsField("date2",QVariant.DateTime) ] )
    path_layers.updateFields() 
    data_sim=pd.read_csv("/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/data/testdataPosi.csv")
    select_d1=data_sim.loc[data_sim['drone'] == 'd1']
    select_d1.sort_values(by='datetime', inplace = True)
    start_row=0
    nb_row=select_d1.shape[0]
    for i in range(start_row,select_d1.shape[0]-1):
        starpoint=select_d1.iloc[i]
        endpoint=select_d1.iloc[i+1]
        formatted_start = datetime.datetime.strptime(starpoint["datetime"],'%Y-%m-%d %H:%M:%S.%f')
        formatted_end = datetime.datetime.strptime(endpoint["datetime"],'%Y-%m-%d %H:%M:%S.%f')
        datestart=QDateTime(formatted_start.year, formatted_start.month, formatted_start.day, formatted_start.hour, formatted_start.minute, formatted_start.second)
        dateend=QDateTime(formatted_end.year, formatted_end.month, formatted_end.day, formatted_end.hour, formatted_end.minute, formatted_end.second)
        fet = QgsFeature()
        fet.setGeometry(QgsGeometry.fromPolylineXY([QgsPointXY(starpoint['x'],starpoint['y']), QgsPointXY(endpoint['x'],endpoint['y'])]))
        fet.setAttributes([starpoint['drone'], starpoint['x'], starpoint['y'], datestart, endpoint['x'], endpoint['y'], dateend])
        path_layers.addFeature(fet)
    path_layers.commitChanges()
    tempo_vl=path_layers.temporalProperties()
    tempo_vl.setIsActive(True)
    tempo_vl.setStartField("date2")
    tempo_vl.setMode(Qgis.VectorTemporalMode(1))
    return path_layers


 
               
class CustomWind(QMainWindow):
    def __init__(self,layers):
        QMainWindow.__init__(self)
        self.canvas = QgsMapCanvas()
        self.temporal_controller = QgsTemporalControllerWidget()
        self.canvas.setTemporalController(self.temporal_controller.temporalController())
        
        self.layers=layers
        self.canvas.setDestinationCrs(layers[-1].crs())
        self.canvas.setExtent(layers[-1].extent())
        self.canvas.setLayers(layers)
        self.setCentralWidget(self.canvas)
        self.setMenuWidget(self.temporal_controller)
        temporalController = self.canvas.temporalController()
        self.toolbar = self.addToolBar("Canvas actions")
        self.toolPan = QgsMapToolPan(self.canvas)
        
        self.actionPushBut = QAction("Delete data", self)
        self.actionPushBut.setCheckable(False)
        self.actionPushBut.triggered.connect(self.del_push)
        self.toolbar.addAction(self.actionPushBut)
        self.toolPan.setAction(self.actionPushBut)
        
        self.actionPushButbis = QAction("replace data", self)
        self.actionPushButbis.setCheckable(False)
        self.actionPushButbis.triggered.connect(self.replace_push)
        self.toolbar.addAction(self.actionPushButbis)
        self.toolPan.setAction(self.actionPushButbis)
        
        self.actionPushDelRep = QAction("Del/Rep", self)
        self.actionPushDelRep.setCheckable(False)
        self.actionPushDelRep.triggered.connect(self.delrep_push)
        self.toolbar.addAction(self.actionPushDelRep)
        self.toolPan.setAction(self.actionPushDelRep)
        
        
        
        # self.push_button = QPushButton("Click me!")
        # self.push_button.clicked.connect(self.f)
        
        
        
    def del_push(self):
        drone_lay=self.layers[0]
        drone_lay.startEditing()
        for feat in drone_lay.getFeatures():
            drone_lay.deleteFeature(feat.id())
        drone_lay.commitChanges()
        path_lay=self.layers[1]
        path_lay.startEditing()
        for feat in path_lay.getFeatures():
            path_lay.deleteFeature(feat.id())
        path_lay.commitChanges()
        return
               
    def replace_push(self):
        drone_lay=self.layers[0]
        pr = drone_lay.dataProvider()
        drone_lay.startEditing()
        pr.addAttributes( [ QgsField("drone", QVariant.String),
                    QgsField("X",  QVariant.Int),
                    QgsField("Y",  QVariant.Int),
                   QgsField("date",QVariant.DateTime) ] )
                   
                   
        data_sim=pd.read_csv("/home/companion/PlanSys/testdataPosi.csv")
        select_d1=data_sim.loc[data_sim['drone'] == 'd1']
        for index,row in select_d1.iterrows():
            formatted = datetime.datetime.strptime(row["datetime"],'%Y-%m-%d %H:%M:%S.%f')
            newdatebis=QDateTime(formatted.year, formatted.month, formatted.day, formatted.hour, formatted.minute, formatted.second)
            fet = QgsFeature()
            fet.setAttributes([row['drone'],row['x'],row['y'],newdatebis])
            fet.setGeometry( QgsGeometry.fromPointXY(QgsPointXY(row['x'],row['y'])) )
            pr.addFeatures( [ fet ] )               

        # Commit changes
        drone_lay.commitChanges()
        return
        
    def delrep_push(self):
        drone_lay=self.layers[0]
        pr = drone_lay.dataProvider()
        drone_lay.startEditing()
        for feat in drone_lay.getFeatures():
            drone_lay.deleteFeature(feat.id())
        pr.addAttributes( [ QgsField("drone", QVariant.String),
                    QgsField("X",  QVariant.Int),
                    QgsField("Y",  QVariant.Int),
                   QgsField("date",QVariant.DateTime) ] )
                   
                   
        data_sim=pd.read_csv("/home/companion/PlanSys/testdataPosi.csv")
        select_d1=data_sim.loc[data_sim['drone'] == 'd1']
        for index,row in select_d1.iterrows():
            formatted = datetime.datetime.strptime(row["datetime"],'%Y-%m-%d %H:%M:%S.%f')
            newdatebis=QDateTime(formatted.year, formatted.month, formatted.day, formatted.hour, formatted.minute, formatted.second)
            fet = QgsFeature()
            fet.setAttributes([row['drone'],row['x'],row['y'],newdatebis])
            fet.setGeometry( QgsGeometry.fromPointXY(QgsPointXY(row['x'],row['y'])) )
            pr.addFeatures( [ fet ] )       
        drone_lay.commitChanges()
               
              
    # Refresh = add without any duplicate  
               



class CustomWindBis(CustomWind):

    def __init__(self,layers):
        super().__init__(layers)
        self.actionShowlayer = QAction("Hide Drone", self)
        self.actionShowlayer.setCheckable(True)
        self.actionShowlayer.triggered.connect(self.show_hide)
        self.toolbar.addAction(self.actionShowlayer)
        self.toolPan.setAction(self.actionShowlayer)
        
        self.actionShowlayerbis = QAction("Hide Path", self)
        self.actionShowlayerbis.setCheckable(True)
        self.actionShowlayerbis.triggered.connect(self.show_hidebis)
        self.toolbar.addAction(self.actionShowlayerbis)
        self.toolPan.setAction(self.actionShowlayerbis)
        
        
        self.current_data_index=0#-1
        self.actionPushButbis.setText("Refresh Data")
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(15001)
        
    def refresh(self):
        data_sim=pd.read_csv("/home/companion/PlanSys/testdataPosi.csv")
        select_d1=data_sim.loc[data_sim['drone'] == 'd1']
        drone_lay=self.layers[0]
        path_lay=self.layers[1]
        # drone_lay.startEditing()
        # #select_d1.iloc[self.current_data_index:].iterrows()# with self.current_data_index=0
        # init_index=self.current_data_index
        # for index,row in select_d1.iloc[self.current_data_index:].iterrows():
            # #if index>init_index:
            # formatted = datetime.datetime.strptime(row["datetime"],'%Y-%m-%d %H:%M:%S.%f')
            # newdatebis=QDateTime(formatted.year, formatted.month, formatted.day, formatted.hour, formatted.minute, formatted.second)
            # fet = QgsFeature()
            # fet.setAttributes([row['drone'],row['x'],row['y'],newdatebis])
            # fet.setGeometry( QgsGeometry.fromPointXY(QgsPointXY(row['x'],row['y'])) )
            # drone_lay.addFeature(fet)
            # init_index=index
        # # Commit changes
        # drone_lay.commitChanges()
        
        drone_lay.startEditing()
        path_lay.startEditing()
        # data_sim=pd.read_csv("/home/companion/PlanSys/testdataPosi.csv")
        # select_d1=data_sim.loc[data_sim['drone'] == 'd1']
        select_d1.sort_values(by='datetime', inplace = True)
        # start_row=0
        # nb_row=select_d1.shape[0]
        for i in range(self.current_data_index,select_d1.shape[0]-1):
            starpoint=select_d1.iloc[i]
            endpoint=select_d1.iloc[i+1]
            formatted_start = datetime.datetime.strptime(starpoint["datetime"],'%Y-%m-%d %H:%M:%S.%f')
            formatted_end = datetime.datetime.strptime(endpoint["datetime"],'%Y-%m-%d %H:%M:%S.%f')
            datestart=QDateTime(formatted_start.year, formatted_start.month, formatted_start.day, formatted_start.hour, formatted_start.minute, formatted_start.second)
            dateend=QDateTime(formatted_end.year, formatted_end.month, formatted_end.day, formatted_end.hour, formatted_end.minute, formatted_end.second)
            fet = QgsFeature()
            fet.setGeometry(QgsGeometry.fromPolylineXY([QgsPointXY(starpoint['x'],starpoint['y']), QgsPointXY(endpoint['x'],endpoint['y'])]))
            fet.setAttributes([starpoint['drone'], starpoint['x'], starpoint['y'], datestart, endpoint['x'], endpoint['y'], dateend])
            path_lay.addFeature(fet)
            fet = QgsFeature()
            fet.setAttributes([endpoint['drone'],endpoint['x'],endpoint['y'],dateend])
            fet.setGeometry( QgsGeometry.fromPointXY(QgsPointXY(endpoint['x'],endpoint['y'])) )
            drone_lay.addFeature(fet)
        path_lay.commitChanges()
        drone_lay.commitChanges()
        self.timer.start(15001)
        return 0
    
    def show_hide( self, checked ):
        print(QgsProject.instance().layerTreeRoot().findLayer(self.layers[0]))
        button_layer=self.layers[0]
        if not checked:
            print("Show")
            if self.layers[0] not in self.canvas.layers():
                self.canvas.setLayers([self.layers[0]]+self.canvas.layers())
        else:
            print("Hide")
            tmp_layers=self.canvas.layers()
            tmp_layers.remove(self.layers[0])
            self.canvas.setLayers(tmp_layers)
            
    def show_hidebis( self, checked ):
        print(QgsProject.instance().layerTreeRoot().findLayer(self.layers[1]))
        button_layer=self.layers[1]
        if not checked:
            print("Show")
            if self.layers[1] not in self.canvas.layers():
                self.canvas.setLayers([self.layers[1]]+self.canvas.layers())
        else:
            print("Hide")
            tmp_layers=self.canvas.layers()
            tmp_layers.remove(self.layers[1])
            self.canvas.setLayers(tmp_layers)
    
    def delrep_push(self):
        return 0
 





        
        




def main(args=None):
    rclpy.init()
    qtwindow_node=WindowQgis()
    #executor = SingleThreadedExecutor()
    #executor.add_node(qtwindow_node)
    qgs = QgsApplication([], False)
    qgs.initQgis()
    QgsApplication.setPrefixPath("/usr/", True)

    #lidar_layer=gen_lidarlayer()
    #zone_layer=gen_zonelayer()
    #drone_layers=gen_dronelayer()


    #add_layer_toproject(lidar_layer,"Cloud")
    #add_layer_toproject(zone_layer,"Vectorzone")
    #add_layer_toproject(drone_layers,"VectorPoint")



    #cusw=CustomWind([drone_layers,zone_layer,lidar_layer],)
    
    window = QMainWindow()
    window.setWindowTitle("TestWindow")
    window.setStyleSheet("background-color: yellow;")
    window.show()
    
    
    print("Test0")
    #cusw.show()
    #cuw2=CustomWind2()
    #cuw2.show()
    #import sys
    print(sys.path)

    qgs.exit()
    #qgs.exitQgis()
    print("Test1print")
    # # qtwindow_node.get_logger().info("Test1log")
    rclpy.spin(qtwindow_node)
    qtwindow_node.destroy_node()

    rclpy.shutdown()

# if __name__ == "__main__":
    # main()

# main()


qgs = QgsApplication([], False)
qgs.initQgis()
QgsApplication.setPrefixPath("/usr/", True)

lidar_layer_out=gen_lidarlayer()
zone_layer_out=gen_zonelayer()
drone_layer_out=gen_dronelayer()
path_layer_out=gen_pathlayer()


add_layer_toproject(lidar_layer_out,"Cloud")
add_layer_toproject(zone_layer_out,"Vectorzone")
add_layer_toproject(path_layer_out,"VectorLine")
add_layer_toproject(drone_layer_out,"VectorPoint")

cuswin=CustomWindBis([drone_layer_out,path_layer_out,zone_layer_out,lidar_layer_out])
cuswin.show()

qgs.exit()

