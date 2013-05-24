"""
  modulair_app_menu_button
  This button is the widget to be used for all the application entries.
  
  @author: Andy Tien
"""

from PySide.QtGui import QWidget, QPixmap, QLabel, QVBoxLayout

class ModulairAppButton(QWidget):
  def __init__(self, app_tag, app_image_path, app_description_path):
    QWidget.__init__(self)
    #App tag
    self.app_tag_ = QLabel(app_tag)
    #App image
    app_image = QPixmap()
    app_image.load(app_image_path)
    self.app_image_ = QLabel()
    self.app_image_.setPixmap(app_image)
    #App description
    try:
      f = open(app_description_path, 'r')  
      self.app_description_ = f.read()
      f.close()
    except:
      print "Error opening description. Quitting."
    
    self.setToolTip(self.app_description_)
    #Layout the child widgets
    self.child_layout_ = QVBoxLayout()
    self.child_layout_.addWidget(self.app_image_)
    self.child_layout_.addWidget(self.app_tag_)
    
    self.setLayout(self.child_layout_)