#include "qsidebar.h"

QSideBar::QSideBar(QWidget *parent)
: QWidget(parent), m_currentToolBt(NULL)
{
    m_parentLayout = new QHBoxLayout(this);
    m_parentLayout->setContentsMargins(0, 0, 0, 0);
    m_barWidget = new QWidget(this);
    m_barWidget->setFixedWidth(64);
    m_barLayout = new QVBoxLayout(m_barWidget);
    m_barLayout->setContentsMargins(0, 0, 0, 0);
    m_barLayout->setSpacing(0);

    m_blankWidget = new QWidget(this);
    //m_blankWidget->setVisible(false);

    m_parentLayout->addWidget(m_barWidget);
    m_parentLayout->addWidget(m_blankWidget);

    QSpacerItem *pItem = new QSpacerItem(10, 10, QSizePolicy::Preferred, QSizePolicy::Expanding);
    m_barLayout->addSpacerItem(pItem);
}

QSideBar::~QSideBar()
{

}


//QToolButton *QSideBar::addTooButton(const QString &strName, const QString &strToolTip, const QIcon &icon)
//{
//    QToolButton *pToolBt = new QToolButton(m_barWidget);
//    pToolBt->setText(strName);
//    pToolBt->setToolTip(strToolTip);
//    pToolBt->setIcon(icon);
//    pToolBt->setIconSize(QSize(64, 64));
//    pToolBt->setCheckable(true);
//    connect(pToolBt, &QToolButton::clicked, this, &QSideBar::showWidget);
//    m_buttonGroup.addButton(pToolBt);

//    m_barLayout->insertWidget(m_barLayout->count() - 1, pToolBt);
//    return pToolBt;
//}


QToolButton *QSideBar::addTooButton(const QString &strName, const QString &strToolTip)
{
    QToolButton *pToolBt = new QToolButton(m_barWidget);
    pToolBt->setText(strName);
    pToolBt->setToolTip(strToolTip);
    //pToolBt->setIcon(icon);
    pToolBt->setIconSize(QSize(64, 64));
    pToolBt->setCheckable(true);
    connect(pToolBt, &QToolButton::clicked, this, &QSideBar::showWidget);
    m_buttonGroup.addButton(pToolBt);

    m_barLayout->insertWidget(m_barLayout->count() - 1, pToolBt);
    return pToolBt;
}


void QSideBar::setToolWidget(QToolButton *pToolBt, QWidget *pWidget)
{
    m_toolWidget[pToolBt] = pWidget;
    if (pWidget)
    {
        m_parentLayout->addWidget(pWidget);
        pWidget->setVisible(false);
    }
}

void QSideBar::showWidget()
{
    m_blankWidget->setVisible(false);
    auto it = m_toolWidget[m_currentToolBt];
    if (it)
        it->setVisible(false);

    QToolButton *pTool = qobject_cast<QToolButton*>(sender());
    if (pTool)
    {
        m_currentToolBt = pTool;

        auto it = m_toolWidget[pTool];
        if (it)
            it->setVisible(true);
        else
            m_blankWidget->setVisible(true);
    }
}
