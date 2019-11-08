#include "dataload_ulog.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QWidget>
#include <QSettings>
#include <QProgressDialog>
#include <QMainWindow>
#include "selectlistdialog.h"
#include "ulog_parser.h"
#include "ulog_parameters_dialog.h"

DataLoadULog::DataLoadULog(): _main_win(nullptr)
{
    foreach(QWidget *widget, qApp->topLevelWidgets())
    {
        if(widget->inherits("QMainWindow"))
        {
            _main_win = widget;
            break;
        }
    }
}

const std::vector<const char*> &DataLoadULog::compatibleFileExtensions() const
{
    static  std::vector<const char*> extensions = { "ulg" };
    return extensions;
}


PlotDataMapRef DataLoadULog::readDataFromFile(const QString &file_name, bool)
{
    PlotDataMapRef plot_data;
    ULogParser parser( file_name.toStdString() );

    const auto& timeseries_map = parser.getTimeseriesMap();

    for( const auto& it: timeseries_map)
    {
        const std::string& sucsctiption_name =  it.first;
        const ULogParser::Timeseries& timeseries = it.second;

        for (const auto& data: timeseries.data )
        {
            std::string series_name = sucsctiption_name + data.first;

            auto series = plot_data.addNumeric( series_name );

            for( size_t i=0; i < data.second.size(); i++ )
            {
                double msg_time = static_cast<double>(timeseries.timestamps[i]) * 0.000001;
                PlotData::Point point( msg_time, data.second[i] );
                series->second.pushBack( point );
            }
        }
    }


    //PlotData qw = plot_data.numeric["vehicle_attitude/q0"];

    auto q0 = plot_data.numeric.find("vehicle_attitude/q.00");
    auto q1 = plot_data.numeric.find("vehicle_attitude/q.01");
    auto q2 = plot_data.numeric.find("vehicle_attitude/q.02");
    auto q3 = plot_data.numeric.find("vehicle_attitude/q.03");


    auto roll_iterator = plot_data.addNumeric("vehicle_attitude/roll");
    auto pitch_iterator = plot_data.addNumeric("vehicle_attitude/pitch");
    auto yaw_iterator = plot_data.addNumeric("vehicle_attitude/yaw");

    for (int i = 0; i < q0->second.size(); i++) {

        double a = q0->second.at(i).y;
        double b = q1->second.at(i).y;
        double c = q2->second.at(i).y;
        double d = q3->second.at(i).y;
        double aa = a * a;
        double ab = a * b;
        double ac = a * c;
        double ad = a * d;
        double bb = b * b;
        double bc = b * c;
        double bd = b * d;
        double cc = c * c;
        double cd = c * d;
        double dd = d * d;

        double dcm_0_0 = aa + bb - cc - dd;
        double dcm_0_1 = 2 * (bc - ad);
        double dcm_0_2 = 2 * (ac + bd);
        double dcm_1_0 = 2 * (bc + ad);
        double dcm_1_1 = aa - bb + cc - dd;
        double dcm_1_2 = 2 * (cd - ab);
        double dcm_2_0 = 2 * (bd - ac);
        double dcm_2_1 = 2 * (ab + cd);
        double dcm_2_2 = aa - bb - cc + dd;

        double phi_val = atan2(dcm_2_1, dcm_2_2);
        double theta_val = asin(-dcm_2_0);
        double psi_val = atan2(dcm_1_0, dcm_0_0);

        if (fabs(theta_val - M_PI / 2) < 1.0e-3) {
            phi_val = 0;
            psi_val = atan2(dcm_1_2,dcm_0_2);

        } else if (fabs(theta_val + M_PI / 2) < 1.0e-3) {
            phi_val = 0;
            psi_val = atan2(-dcm_1_2, -dcm_0_2);
        }

        
        roll_iterator->second.pushBack(PlotData::Point(q0->second.at(i).x, phi_val));
        pitch_iterator->second.pushBack(PlotData::Point(q0->second.at(i).x, theta_val));
        yaw_iterator->second.pushBack(PlotData::Point(q0->second.at(i).x, psi_val));
    }
    // create ground speed
    auto vx = plot_data.numeric.find("vehicle_local_position/vx");
    auto vy = plot_data.numeric.find("vehicle_local_position/vy");

    auto ground_speed_iterator = plot_data.addNumeric("vehicle_local_position/ground_speed");

    for (int i = 0; i < vx->second.size(); i++) {
        double ground_speed = sqrt(vx->second.at(i).y * vx->second.at(i).y + vy->second.at(i).y * vy->second.at(i).y);
        PlotData::Point point( vx->second.at(i).x, ground_speed );
        ground_speed_iterator->second.pushBack(PlotData::Point( vx->second.at(i).x, ground_speed ));

    }

    QVector<QString> control_mode_fields;

    control_mode_fields.append(QString("tilt_aligned"));
    control_mode_fields.append(QString("yaw_aligned"));
    control_mode_fields.append(QString("gps"));
    control_mode_fields.append(QString("opt_flow"));
    control_mode_fields.append(QString("mag_hdg"));
    control_mode_fields.append(QString("mag_3D"));
    control_mode_fields.append(QString("mag_dec"));
    control_mode_fields.append(QString("in_air"));
    control_mode_fields.append(QString("wind"));
    control_mode_fields.append(QString("baro_hgt"));
    control_mode_fields.append(QString("rng_hgt"));
    control_mode_fields.append(QString("gps_hgt"));
    control_mode_fields.append(QString("ev_pos"));
    control_mode_fields.append(QString("ev_yaw"));
    control_mode_fields.append(QString("ev_hgt"));
    control_mode_fields.append(QString("beta"));
    control_mode_fields.append(QString("mag_field"));
    control_mode_fields.append(QString("fixed_wing"));
    control_mode_fields.append(QString("mag_fault"));
    control_mode_fields.append(QString("airspeed"));
    control_mode_fields.append(QString("gnd_effect"));
    control_mode_fields.append(QString("rng_stuck"));
    control_mode_fields.append(QString("gps_yaw"));
    control_mode_fields.append(QString("mag_aligned"));

    std::map<std::string, std::unordered_map<std::string, PlotData>::iterator> iterator_map;

    for (int i = 0; i < control_mode_fields.size(); i++) {
        iterator_map[control_mode_fields.at(i).toStdString()] = plot_data.addNumeric(std::string("estimator_ctrl_flags/") +control_mode_fields.at(i).toStdString());
    }

    auto control_flags = plot_data.numeric.find("estimator_status/control_mode_flags");

    double val;
    for (int i = 0; i < control_flags->second.size(); i++) {
        for (int j = 0; j < control_mode_fields.size(); j++) {

            if ((uint32_t)control_flags->second.at(i).y & (1 << j)) {
                val = 1;
            } else {
                val = 0;
            }

            PlotData::Point point( control_flags->second.at(i).x, val );
            iterator_map[control_mode_fields.at(j).toStdString()]->second.pushBack(point);
        }
    }

    auto filter_fault_flags = plot_data.numeric.find("estimator_status/filter_fault_flags");

    QVector<QString> filter_fault_fields;

    filter_fault_fields.append(QString("mag_x"));
    filter_fault_fields.append(QString("mag_y"));
    filter_fault_fields.append(QString("mag_z"));

    filter_fault_fields.append(QString("heading"));
    filter_fault_fields.append(QString("declination"));

    filter_fault_fields.append(QString("airspeed"));
    filter_fault_fields.append(QString("beta"));

    filter_fault_fields.append(QString("flow_x"));
    filter_fault_fields.append(QString("flow_y"));

    filter_fault_fields.append(QString("v_north"));
    filter_fault_fields.append(QString("v_east"));
    filter_fault_fields.append(QString("v_down"));

    filter_fault_fields.append(QString("pos_north"));
    filter_fault_fields.append(QString("pos_east"));
    filter_fault_fields.append(QString("pos_down"));

    filter_fault_fields.append(QString("d_vel_bias"));

    for (int i = 0; i < filter_fault_fields.size(); i++) {
        iterator_map[filter_fault_fields.at(i).toStdString()] = plot_data.addNumeric(std::string("estimator_fault_flags/") +filter_fault_fields.at(i).toStdString());
    }

    for (int i = 0; i < filter_fault_flags->second.size(); i++) {
        for (int j = 0; j < filter_fault_fields.size(); j++) {

            if ((uint32_t)filter_fault_flags->second.at(i).y & (1 << j)) {
                val = 1;
            } else {
                val = 0;
            }

            PlotData::Point point( filter_fault_flags->second.at(i).x, val );
            iterator_map[filter_fault_fields.at(j).toStdString()]->second.pushBack(point);
        }
    }

    auto solution_status_flags = plot_data.numeric.find("estimator_status/solution_status_flags");

    QVector<QString> solution_status_fields;

    solution_status_fields.append(QString("attitude_good"));
    solution_status_fields.append(QString("hor_vel_good"));
    solution_status_fields.append(QString("vert_vel_good"));
    solution_status_fields.append(QString("hor_pos_rel_good"));
    solution_status_fields.append(QString("hor_pos_abs_good"));
    solution_status_fields.append(QString("vert_pos_abs_good"));
    solution_status_fields.append(QString("hagl_good"));
    solution_status_fields.append(QString("const_pos_mode"));
    solution_status_fields.append(QString("rel_pos_mode"));
    solution_status_fields.append(QString("abs_pos_mode"));
    solution_status_fields.append(QString("gps_glitch"));
    solution_status_fields.append(QString("bad_accel"));

    for (int i = 0; i < solution_status_fields.size(); i++) {
        iterator_map[solution_status_fields.at(i).toStdString()] = plot_data.addNumeric(std::string("estimator_solution_status/") +solution_status_fields.at(i).toStdString());
    }

    for (int i = 0; i < solution_status_flags->second.size(); i++) {
        for (int j = 0; j < solution_status_fields.size(); j++) {

            if ((uint32_t)solution_status_flags->second.at(i).y & (1 << j)) {
                val = 1;
            } else {
                val = 0;
            }

            PlotData::Point point( solution_status_flags->second.at(i).x, val );
            iterator_map[solution_status_fields.at(j).toStdString()]->second.pushBack(point);
        }
    }

//----------------------------------------
    auto gps_check_fail_flags = plot_data.numeric.find("estimator_status/gps_check_fail_flags");

    QVector<QString> gps_check_fail_flags_fields;

    gps_check_fail_flags_fields.append(QString("gps_fix"));
    gps_check_fail_flags_fields.append(QString("min_sat_count"));
    gps_check_fail_flags_fields.append(QString("min_gdop"));
    gps_check_fail_flags_fields.append(QString("max_horiz_err"));
    gps_check_fail_flags_fields.append(QString("max_vert_err"));
    gps_check_fail_flags_fields.append(QString("max_spd_err"));
    gps_check_fail_flags_fields.append(QString("max_horiz_drift"));
    gps_check_fail_flags_fields.append(QString("max_vert_drift"));
    gps_check_fail_flags_fields.append(QString("max_horiz_spd_err"));
    gps_check_fail_flags_fields.append(QString("max_vert_spd_err"));

    for (int i = 0; i < gps_check_fail_flags_fields.size(); i++) {
        iterator_map[gps_check_fail_flags_fields.at(i).toStdString()] = plot_data.addNumeric(std::string("gps_check_fail_flags/") +gps_check_fail_flags_fields.at(i).toStdString());
    }

    for (int i = 0; i < gps_check_fail_flags->second.size(); i++) {
        for (int j = 0; j < gps_check_fail_flags_fields.size(); j++) {

            if ((uint32_t)gps_check_fail_flags->second.at(i).y & (1 << j)) {
                val = 1;
            } else {
                val = 0;
            }

            PlotData::Point point( gps_check_fail_flags->second.at(i).x, val );
            iterator_map[gps_check_fail_flags_fields.at(j).toStdString()]->second.pushBack(point);
        }
    }
//
    auto innovation_check_flags = plot_data.numeric.find("estimator_status/innovation_check_flags");

    QVector<QString> innovation_fields;

    innovation_fields.append(QString("vel"));
    innovation_fields.append(QString("hor_pos"));
    innovation_fields.append(QString("ver_pos"));
    innovation_fields.append(QString("vel"));
    innovation_fields.append(QString("vel"));
    innovation_fields.append(QString("vel"));


    ULogParametersDialog* dialog = new ULogParametersDialog( parser, _main_win );
    dialog->setWindowTitle( QString("ULog file %1").arg(file_name) );
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->restoreSettings();
    dialog->show();

    return plot_data;
}

DataLoadULog::~DataLoadULog()
{

}

QDomElement DataLoadULog::xmlSaveState(QDomDocument &doc) const
{
    QDomElement elem = doc.createElement("no_params");
    return elem;
}

bool DataLoadULog::xmlLoadState(QDomElement&)
{
    return true;
}
