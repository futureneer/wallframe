#ifndef MODULAIR_INFO_BAR_H
#define MODULAIR_INFO_BAR_H

#include "QtGui"
#include <QtGui/QApplication>
#include <QObject>

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <modulair_common/modulair_common.h>


namespace modulair{

  class UserIcon : public QWidget{
    Q_OBJECT
  public:
    UserIcon(QWidget *parent, int id, QString text, QList<QPixmap> &assets) : QWidget(parent){
      id_ = id;
      icon_assets_ = assets;

      state_ = ICO_STATE_IDLE;

      QFont textFont;
      textFont.setPixelSize(I_HEIGHT/7);
      textFont.setBold(true);

      // Icon Background
      icon_ = new QLabel(this);
      icon_->resize(I_WIDTH,I_HEIGHT);
      icon_->move(0,0);
      icon_->setPixmap(icon_assets_[1]);
      icon_->setScaledContents(true);
      icon_->show();

      text_ = new QLabel(this);
      text_->resize(I_WIDTH-I_SPACER*2,I_HEIGHT/7);
      text_->move(I_SPACER,I_SPACER*2);
      text_->setFont(textFont);
      text_->setText(text);
      text_->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
      text_->setStyleSheet("color:#ffffff");
      text_->show();

      image_ = new QLabel(this);
      image_->resize(I_WIDTH-I_SPACER*2,I_HEIGHT*3/4);
      image_->move(I_SPACER,I_HEIGHT/5);
      image_->setPixmap(icon_assets_[4]);
      image_->setScaledContents(true);
      image_->show();
    };

    ~UserIcon(){};

    // Member Functions
   void eval()
   {
     QPixmap imagePixmap;
     QPixmap iconPixmap;
     if(state_ != state_last_){
      switch(state_){
       case ICO_STATE_IDLE:             
        break;
       case ICO_STATE_APPROACHED:
         icon_->setPixmap(icon_assets_[1]);
         image_->setPixmap(icon_assets_[15]);   
         break;
       case ICO_STATE_ACTIVE_WAITING:
         icon_->setPixmap(icon_assets_[1]);
         image_->setPixmap(icon_assets_[15]); 
         break;
       case ICO_STATE_ACTIVE_RETRY:
         icon_->setPixmap(icon_assets_[1]);
         image_->setPixmap(icon_assets_[14]); 
         break;
       case ICO_STATE_ACTIVE:
         icon_->setPixmap(icon_assets_[1]);
         image_->setPixmap(icon_assets_[17]);         
         break;
       case ICO_STATE_PRIME: 
         icon_->setPixmap(icon_assets_[9]);
         image_->setPixmap(icon_assets_[19]);           
         break;
        case ICO_STATE_CENTER:
          icon_->setPixmap(icon_assets_[2]);
          image_->setPixmap(icon_assets_[18]);
          break;
        case ICO_STATE_BACK:
          icon_->setPixmap(icon_assets_[2]);
          image_->setPixmap(icon_assets_[20]);
          break;
      }
    }
    state_last_ = state_;
  };

  // Members
  int state_;

  private:
    // Members
    int id_;
    QLabel *icon_;
    QLabel *text_;
    QLabel *image_;
    int state_last_;
    QList<QPixmap> icon_assets_;
  };

  typedef std::map<int,UserIcon*> UserIconPtrMap;

  class UserMinIcon : public LairWidget{
    Q_OBJECT
  public:
    UserMinIcon(QWidget *parent, int id, QString text) : 
    LairWidget(parent)
    {
     id_ = id;
     state_ = ICO_STATE_IDLE;

     QFont textFont;
     textFont.setPixelSize(IM_HEIGHT/2);
     textFont.setBold(true);

     text_ = new QLabel(this);
     text_->resize(I_WIDTH,IM_HEIGHT-IM_SPACER*2);
     text_->move(0,IM_SPACER);
     text_->setFont(textFont);
     text_->setText(text);
     text_->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
     text_->setAutoFillBackground(true);
        // text_->setStyleSheet(lair::APP_BUTTON_STYLE);
     text_->setStyleSheet("background-color:#ff8a00;color:#ffffff;border:4x solid #ffffff");
     text_->show();

     this->setStyleSheet("background-color:#ff8a00;color:#ffffff;border:4px solid #ffffff");

   };
    ~UserMinIcon(){};
     // Member Functions
    void eval()
    {
     if(state_ != state_last_){
      switch(state_){
       case ICO_STATE_IDLE:             
        break;
       case ICO_STATE_APPROACHED:
         text_->setStyleSheet("background-color:#ff8a00;color:#ffffff;border:4x solid #ffffff");
         this->setStyleSheet("background-color:#ff8a00;color:#ffffff;border:4x solid #ffffff");
         break;
       case ICO_STATE_ACTIVE_WAITING:
         text_->setStyleSheet("background-color:#8fa377;color:#ffffff;border:4x solid #ffffff");
         this->setStyleSheet("background-color:#8fa377;color:#ffffff;border:4x solid #ffffff");
         break;
       case ICO_STATE_ACTIVE_RETRY:
         text_->setStyleSheet("background-color:#a26b6b;color:#ffffff;border:4x solid #ffffff");
         this->setStyleSheet("background-color:#a26b6b;color:#ffffff;border:4x solid #ffffff");
         break;
       case ICO_STATE_ACTIVE: 
         text_->setStyleSheet("background-color:#ff8a00;color:#ffffff;border:4x solid #ffffff");      
         this->setStyleSheet("background-color:#ff8a00;color:#ffffff;border:4x solid #ffffff");
         break;
       case ICO_STATE_PRIME:  
         text_->setStyleSheet("background-color:#25a5cf;color:#ffffff;border:4x solid #ffffff");        
         this->setStyleSheet("background-color:#25a5cf;color:#ffffff;border:4x solid #ffffff");
         break;
       case ICO_STATE_BACK:  
         text_->setStyleSheet("background-color:#c11f1f;color:#ffffff;border:4x solid #ffffff");        
         this->setStyleSheet("background-color:#c11f1f;color:#ffffff;border:4x solid #ffffff");
         break;
       case ICO_STATE_CENTER:  
         text_->setStyleSheet("background-color:#c11f1f;color:#ffffff;border:4x solid #ffffff");        
         this->setStyleSheet("background-color:#c11f1f;color:#ffffff;border:4x solid #ffffff");
         break;
     }
   }
   state_last_ = state_;
  };

    // Members
int state_;

private:
  // Members
  int id_;
  QLabel *text_;
  int state_last_;
};

  typedef std::map<int,UserMinIcon*> UserMinIconPtrMap;

  class UserBar : public LairWidget{
    Q_OBJECT  
  public:
    UserBar(QWidget *parent = NULL,bool useKinect = false);
    ~UserBar(){};
    void updateUserIcons();
    void updateUserMinIcons();
    void readConfigFile();
    int mapIconPos(vct3 in);
  Q_SIGNALS:
    void requestUserData();

  private:
    static const double WKSP_MAX_Z = 3000.0;
    static const double WKSP_MIN_Z = 1500.0;
    static const double WKSP_MAX_Y = 100.0;
    static const double WKSP_MIN_Y = -700.0;
    static const double WKSP_MAX_X = 900.0;
    static const double WKSP_MIN_X = -900.0;
    static const double WKSP_MAX_X_ICON = 1200.0;
    static const double WKSP_MIN_X_ICON = -1200.0;

    double GUI_MAX_X;
    double GUI_MIN_X;
    double GUI_MAX_Y;
    double GUI_MIN_Y;

    QTimer timer_;
    UserPtrMap users_;
    UserIconPtrMap user_icons_;
    UserMinIconPtrMap user_min_icons_;
    bool use_kinect_;
    QString asset_dir_;
    QStringList asset_paths_;
    QList<QPixmap> assets_;
    QWidget* my_parent_;
    QWidget* min_bar_;
    bool minimized_;
    std::map<int,bool> marked_del;
    std::map<int,bool> wait_del;

  private Q_SLOTS:
    void recieveUserData(UserPtrMap data);
    void recieveDiscreteGesture(QMap<int,int> events);
    void pullData();
    void minimize();
    void maximize();
  };

}


#endif
