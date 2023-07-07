from qgis.gui import *
from qgis.core import *
from PyQt5.QtCore import *
from qgis.PyQt.QtWidgets import QAction, QMainWindow
import pandas as pd
import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from threading import Thread

from px4_msgs.msg import VehicleGlobalPosition


def add_layer_toproject(layer,namety):
    if not layer.isValid():
        print(f"{namety} Layer failed to load!")
    else:
        QgsProject.instance().addMapLayer(layer)

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




class CollectorNode(Node):

    def __init__(self):
        super().__init__('qgis_gui')
        self.local_pos_sub = self.create_subscription(VehicleGlobalPosition,'/fmu/vehicle_global_position/out', self.listener_callback_pos,10)
        self.oldtime=0
        self.df=pd.DataFrame([],columns=["drone","x","y","datetime"])
        self.local_pos_sub
        
    def listener_callback_pos(self, msg):
        if msg.timestamp-self.oldtime>5000000: #10 second: 10000000:
            current_time=datetime.datetime.now()
            self.df.loc[len(self.df.index)]=["d1",msg.lon,msg.lat,str(current_time)]
            self.oldtime=msg.timestamp
            self.get_logger().info(f"Longitude:{msg.lon},Latitude:{msg.lat}")

 
 
               
class CustomWind(QMainWindow):
    def __init__(self,layers):
        QMainWindow.__init__(self)
        self.canvas = QgsMapCanvas()
        self.temporal_controller_widg = QgsTemporalControllerWidget()
        self.canvas.setTemporalController(self.temporal_controller_widg.temporalController())
        
        self.executor=SingleThreadedExecutor()
        self.node=CollectorNode()
        self.executor.add_node(self.node)
        
        
        self.layers=layers
        self.canvas.setDestinationCrs(layers[-1].crs())
        self.canvas.setExtent(layers[-1].extent())
        self.canvas.setLayers(layers)
        self.setCentralWidget(self.canvas)
        self.setMenuWidget(self.temporal_controller_widg)
        temporalController = self.canvas.temporalController()
        self.toolbar = self.addToolBar("Canvas actions")
        self.toolPan = QgsMapToolPan(self.canvas)
        
        self.actionPushBut = QAction("Delete data", self)
        self.actionPushBut.setCheckable(False)
        self.actionPushBut.triggered.connect(self.del_push)
        self.toolbar.addAction(self.actionPushBut)
        self.toolPan.setAction(self.actionPushBut)
        
        self.actionPushButbis = QAction("Refresh Data", self)
        self.actionPushButbis.setCheckable(False)
        self.actionPushButbis.triggered.connect(self.refresh)
        self.toolbar.addAction(self.actionPushButbis)
        self.toolPan.setAction(self.actionPushButbis)
        
        
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
        
        
        self.current_data_index=0
        self.previousRow=pd.DataFrame() #Empty dictionary for multiple drone
        self.actionPushButbis.setText("Refresh Data")
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(15001)
        self.thread=Thread(target=self.executor.spin)
    
    
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
    
    
    def refresh(self):
        select_d1=self.node.df.loc[self.node.df['drone'] == 'd1']
        drone_lay=self.layers[0]
        path_lay=self.layers[1]
        drone_lay.startEditing()
        path_lay.startEditing() 
        select_d1.sort_values(by='datetime', inplace = True)
        tmpsubdf=select_d1.iloc[self.current_data_index:]
        for index,row in tmpsubdf.iterrows():
            formatted = datetime.datetime.strptime(row["datetime"],'%Y-%m-%d %H:%M:%S.%f')
            newdatebis=QDateTime(formatted.year, formatted.month, formatted.day, formatted.hour, formatted.minute, formatted.second)
            fet = QgsFeature()
            fet.setAttributes([row['drone'],row['x'],row['y'],newdatebis])
            fet.setGeometry( QgsGeometry.fromPointXY(QgsPointXY(row['x'],row['y'])) )
            drone_lay.addFeature(fet)
            if not self.previousRow.empty:
                formatted_prev = datetime.datetime.strptime(self.previousRow["datetime"],'%Y-%m-%d %H:%M:%S.%f')
                prevdate=QDateTime(formatted.year, formatted.month, formatted.day, formatted.hour, formatted.minute, formatted.second)
                fet = QgsFeature()
                fet.setGeometry(QgsGeometry.fromPolylineXY([QgsPointXY(self.previousRow['x'],self.previousRow['y']), QgsPointXY(row['x'],row['y'])]))
                fet.setAttributes([self.previousRow['drone'], self.previousRow['x'], self.previousRow['y'], prevdate, row['x'], row['y'], newdatebis])
                path_lay.addFeature(fet)
            self.previousRow=row
            self.current_data_index+=1
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
       

def main(args=None):
    rclpy.init()
    QgsApplication.setPrefixPath("/usr/", True)
    qgs=QgsApplication([],False)
    qgs.initQgis()
    
    #lidar_layer_out=gen_lidarlayer()
    zone_layer_out=gen_zonelayer()
    drone_layer_out=gen_dronelayer()
    path_layer_out=gen_pathlayer()


    #add_layer_toproject(lidar_layer_out,"Cloud")
    add_layer_toproject(zone_layer_out,"Vectorzone")
    add_layer_toproject(path_layer_out,"VectorLine")
    add_layer_toproject(drone_layer_out,"VectorPoint")
    
    cuswin=CustomWind([drone_layer_out,path_layer_out,zone_layer_out])
    cuswin.thread.start()
    cuswin.show()

    qgs.exec()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()

