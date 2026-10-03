"""Gazebo sensor extensions, authored from manufacturer specifications.
See SENSORS.md for source versions, calibration limits and approximations.
"""
import math
import xml.etree.ElementTree as E

def tag(parent,name,value):
    e=E.SubElement(parent,name); e.text=str(value); return e

def sensor(parent,name,kind,pose,topic,frame,hz):
    e=E.SubElement(parent,'sensor',name=name,type=kind)
    tag(e,'pose',pose); tag(e,'topic',topic); tag(e,'gz_frame_id',frame)
    tag(e,'always_on','true'); tag(e,'update_rate',hz)
    return e

def lidar(parent,name,pose,topic,frame,hz,nx,ny,lo,hi,rmin,rmax):
    s=sensor(parent,name,'gpu_lidar',pose,topic,frame,hz)
    l=E.SubElement(s,'lidar'); scan=E.SubElement(l,'scan')
    h=E.SubElement(scan,'horizontal')
    # Exclude the duplicate +pi endpoint; nx bins cover a full revolution.
    for k,v in [('samples',nx),('resolution',1),('min_angle',-math.pi),('max_angle',math.pi-2*math.pi/nx)]: tag(h,k,v)
    v=E.SubElement(scan,'vertical')
    for k,x in [('samples',ny),('resolution',1),('min_angle',math.radians(lo)),('max_angle',math.radians(hi))]: tag(v,k,x)
    r=E.SubElement(l,'range')
    for k,v in [('min',rmin),('max',rmax),('resolution',.001)]: tag(r,k,v)
    return s

def add_sensors(robot):
    # Attach to the existing rack root. Fixed-joint lumping transforms sensors
    # along with the rack, while original CAD / ROS TF frames remain untouched.
    g=E.SubElement(robot,'gazebo',reference='rack_base_link')
    lidar(g,'ydlidar_g2','0.0001749995366 0 0.3266 0 0 0.1537539717',
          '/robocup/g2/scan','laser_frame',10,500,1,1,1,.12,12)
    lidar(g,'livox_mid360s','-0.18 0 1.183 3.141592653589793 0 0',
          '/robocup/mid360s/scan','livox_frame',10,1000,20,-7,52,.1,100)
    # Separate color and depth projections; a single RGBD camera would give
    # both streams the same FOV, unlike a D435f.
    for name,kind,topic,frame,fov,width,height,near,far in [
        ('d435f_depth','depth_camera','/robocup/camera/depth/image','camera_optical_frame',87,1280,720,.2,3),
        ('d435f_color','camera','/robocup/camera/color/image','camera_optical_frame',69,1920,1080,.01,100)]:
        s=sensor(g,name,kind,'-0.12495 0 1.27 0 0 0',topic,frame,30)
        tag(s,'optical_frame_id',frame)
        c=E.SubElement(s,'camera'); tag(c,'horizontal_fov',math.radians(fov))
        im=E.SubElement(c,'image'); tag(im,'width',width); tag(im,'height',height); tag(im,'format','R8G8B8')
        clip=E.SubElement(c,'clip'); tag(clip,'near',near); tag(clip,'far',far)
