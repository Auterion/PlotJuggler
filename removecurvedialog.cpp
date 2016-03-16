#include "removecurvedialog.h"
#include "ui_removecurvedialog.h"
#include <QDebug>
#include "plotwidget.h"

RemoveCurveDialog::RemoveCurveDialog(QWidget *parent, std::map<QString, QwtPlotCurve *> &curves_map) :
    QDialog(parent),
    ui(new Ui::RemoveCurveDialog)
{
    ui->setupUi(this);

    std::map<QString, QwtPlotCurve*>::iterator it;
    for(it = curves_map.begin(); it != curves_map.end(); ++it)
    {
        ui->listCurveWidget->addItem( new QListWidgetItem( it->first) );
    }
}

RemoveCurveDialog::~RemoveCurveDialog()
{
    delete ui;
}

void RemoveCurveDialog::on_listCurveWidget_itemClicked(QListWidgetItem *item)
{
    QFont f = item->font();
    f.setStrikeOut( !f.strikeOut() );
    item->setFont( f );
    qDebug() << "on_listWidget_itemClicked";
    item->font().setStrikeOut( true );
}

void RemoveCurveDialog::on_pushButtonRemove_pressed()
{
    PlotWidget* parent = dynamic_cast<PlotWidget*>( this->parentWidget() );
    if( parent ) // this should always be true...
    {
        for(int index = 0; index <ui->listCurveWidget->count(); ++index)
        {
            QListWidgetItem* item = ui->listCurveWidget->item( index );
            if( item->font().strikeOut() && item->isHidden() == false)
            {
                parent->removeCurve( item->text() );
                item->setHidden( true );
            }
        }
    }
    closeIfEmpty();
}

void RemoveCurveDialog::on_pushButtonSelectAll_pressed()
{
    for(int index = 0; index <ui->listCurveWidget->count(); ++index)
    {
        QListWidgetItem* item = ui->listCurveWidget->item( index );
        QFont f = item->font();
        f.setStrikeOut( true );
        item->setFont( f );
    }
}

void RemoveCurveDialog::closeIfEmpty()
{
    bool isEmpty = true;
    for(int index = 0; index <ui->listCurveWidget->count(); ++index)
    {
        QListWidgetItem* item = ui->listCurveWidget->item( index );
        if( item->isHidden() == false)
        {
            isEmpty = false;
            break;
        }
    }
    if( isEmpty ) this->accept();
}
