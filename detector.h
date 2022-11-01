#ifndef DETECTOR_H
#define DETECTER_H

#include <QObject>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QMap>
#include <QVector>
#include <QDebug>
#include <fstream>
enum state{success, error};
enum methods{SQDIFF = 0, SQDIFF_NORMED = 1, CCORR = 2, CORR_NORMED = 3, CCOEFF = 4, CCOEFF_NORMED = 5};
enum frame_state {next,previos};

// информация о найденых объектах
struct object_inf {
    cv::Mat image;
    std::vector<cv::Point> point; // точки в которых содержится объект
    cv::Mat templateMatching_result;
};

struct image_saving_protocol
{
    unsigned int CAMERA_ID;
    unsigned int NUMBER_OF_FRAMES;
    uint64 tmsec;
    cv::Mat frame;
    std::vector<uint8_t> imgbuff;
};

class detector:public QObject{
    Q_OBJECT
public :
    detector(int a);
    void input_data(const cv::Mat &image, methods m); // подача на вход изображения
    void input_data(const std::vector<uint8_t> &buffer, methods m); // подача изображения в другой форме
     std::pair<int, state> parse_data(const std::string &m_fileName, methods m, frame_state state, image_saving_protocol &read_protocol); // путь до файла с видео
    std::pair<int, state> add_template(const cv::Mat &templ_one); // загрузка шаблона
    std::pair<std::vector<int>, state> add_template(std::vector<cv::Mat> &templs); // загрузка нескольких шаблонов
    std::pair<int, state> add_template(std::string &templ_one); // закгрузка пути до файла шаблона
    std::pair<std::vector<int>, state> add_template(std::vector<std::string> &templs); // закгрузка нескольких путей до файлов шаблона

    //  std::pair<int, state> - возвращает номер шаблона в хранлище и состояние загрузки, если error - номер шаблона -1

  private:
    std::map<int, cv::Mat> templ;
    std::vector<uint64> FrameByteIndex;
    int templ_counter = 0;
    int m_b = qRegisterMetaType<image_saving_protocol>("image_saving_protocol");
    int position = 0;

  private:
    void det_img(const cv::Mat &image, methods m);
    std::vector<uint64> Data_Indexing(const QString &m_fileName);

  signals:
    void output_data(object_inf info);
};

#endif
