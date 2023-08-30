// OpenCV includes

// STD includes

// Qt Includes
#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

// Custom includes
#include "FilterOrientationsWidget/FilterOrientationsWidget.hpp"

FilterOrientationsWidget::FilterOrientationsWidget(QWidget* parent) :
    QDialog(parent),
    _optionsList({135, 0, 90, 45}),
    _filter0Idx(nullptr),
    _filter1Idx(nullptr),
    _filter2Idx(nullptr),
    _filter3Idx(nullptr)
{
    initializeDialog();
    setWindowFlags((Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint) & (~Qt::WindowMaximizeButtonHint));
    setWindowTitle("Super-pixel configuration");
}

FilterOrientationsWidget::~FilterOrientationsWidget()
{
    delete _filter0Idx;
    delete _filter1Idx;
    delete _filter2Idx;
    delete _filter3Idx;
}

void FilterOrientationsWidget::getComboBox(
    int initialIdx,
    QComboBox* &outputComboBoxPtr)
{
    outputComboBoxPtr = new QComboBox(this);

    for (int v : _optionsList)
    {
        outputComboBoxPtr->addItem(std::to_string(v).c_str());
    }

    outputComboBoxPtr->setCurrentIndex(initialIdx);
}

QWidget* FilterOrientationsWidget::getGridOfOptions()
{
    QWidget* boxesWidget = new QWidget(this);
    QGridLayout* boxesLayout = new QGridLayout(boxesWidget);

    getComboBox(0, _filter0Idx);
    boxesLayout->addWidget(_filter0Idx, 0, 0);

    getComboBox(1, _filter1Idx);
    boxesLayout->addWidget(_filter1Idx, 0, 1);

    getComboBox(2, _filter2Idx);
    boxesLayout->addWidget(_filter2Idx, 1, 0);

    getComboBox(3, _filter3Idx);
    boxesLayout->addWidget(_filter3Idx, 1, 1);

    saveIndexes();

    boxesWidget->setLayout(boxesLayout);
    return boxesWidget;
}

void FilterOrientationsWidget::initializeDialog()
{
    QVBoxLayout* mainLayout = new QVBoxLayout();

    // We add our grid of 2x2 option lists.
    QWidget* boxesWidget = getGridOfOptions();

    // We create our label with the configuration procedure description.
    // Font with Arial 14, BOLD and Italic
    QFont f("Arial", 12, QFont::Bold, true);

    QString descText = "From one camera to another, the linear polarization\n"
        "filter positions can change. Use this window to change\n"
        "the orientations based on your camera model. Each of\n"
        "the four dropdown lists shown below corresponds to\n"
        "one filter of the super-pixel. Please select the\n"
        "corresponding filter orientation at each position.\n"
        "There cannot be two filters with the same orientation.\n"
        "If two or more elements have the same orientation, an\n"
        "error will occur, and not change will be commited.";
    QLabel* descriptionTitleLabel = new QLabel("Super-pixel filter configuration", this);
    descriptionTitleLabel->setFont(f);
    QLabel* descriptionTextLabel = new QLabel(descText, this);
    descriptionTitleLabel->setAlignment(Qt::AlignJustify);
    descriptionTextLabel->setAlignment(Qt::AlignJustify);

    // We create the Ok and Cancel buttons, and we place them one aside the other.
    QWidget* buttonsWidget = new QWidget(this);
    QHBoxLayout* buttonsLayout = new QHBoxLayout(buttonsWidget);
    QPushButton* ok = new QPushButton("OK", buttonsWidget);
    QPushButton* cancel = new QPushButton("Cancel", buttonsWidget);

    buttonsLayout->addWidget(ok);
    buttonsLayout->addWidget(cancel);
    buttonsWidget->setLayout(buttonsLayout);

    // We add all the widgets to this widget layout
    mainLayout->addWidget(descriptionTitleLabel);
    mainLayout->addWidget(descriptionTextLabel);
    mainLayout->addWidget(boxesWidget);
    mainLayout->addWidget(buttonsWidget);
    /// This constraint avoid the window from resizing.
    mainLayout->setSizeConstraint(QLayout::SetFixedSize);
    setLayout(mainLayout);

    connect(ok,
        &QPushButton::clicked,
        this,
        &FilterOrientationsWidget::onAcceptedDialog);

    connect(cancel,
        &QPushButton::clicked,
        this,
        &FilterOrientationsWidget::onRejectedDialog);
}

void FilterOrientationsWidget::saveIndexes()
{
    _currentIndexes = std::vector<int>({
        _filter0Idx->currentIndex(),
        _filter1Idx->currentIndex(),
        _filter2Idx->currentIndex(),
        _filter3Idx->currentIndex()});
}

void FilterOrientationsWidget::restoreLastIndexes()
{
    _filter0Idx->setCurrentIndex(_currentIndexes[0]);
    _filter1Idx->setCurrentIndex(_currentIndexes[1]);
    _filter2Idx->setCurrentIndex(_currentIndexes[2]);
    _filter3Idx->setCurrentIndex(_currentIndexes[3]);
}

bool FilterOrientationsWidget::areThereDupplicates(std::vector<int> vec)
{
    int currIdx = 0;
    bool retVal = false;
    for (size_t i = 0; i < vec.size(); i++)
    {
        int counter = 0;
        for (size_t j = currIdx; j < vec.size(); j++)
        {
            if (vec[j] == vec[i])
            {
                counter++;
            }

        }
        // At each iteration we check if there is a dupplicate.
        if (counter > 1)
        {
            retVal = true;
            break;
        }
        currIdx++;
    }
    return retVal;
}

void FilterOrientationsWidget::showDialog()
{
    // We first save the indexes to be sure we can restore them.
    saveIndexes();
    exec();
    std::vector<int> selection({
        _filter0Idx->currentIndex(),
        _filter1Idx->currentIndex(),
        _filter2Idx->currentIndex(),
        _filter3Idx->currentIndex()
    });

    if (areThereDupplicates(selection))
    {
        QMessageBox::critical(this,
            "Super-pixel configuration",
            "You have selected the same orientation for several pixels. "
            "The previous configuration will be loaded",
            QMessageBox::Ok);
        restoreLastIndexes();
    }
    else
    {
        saveIndexes();
        emit orientationsUpdated(getPixelsMap());
    }
}

void FilterOrientationsWidget::keyPressEvent(QKeyEvent *e)
{
    if(e->key() != Qt::Key_Escape)
    {
        QDialog::keyPressEvent(e);
    }
    else
    {
        restoreLastIndexes();
        reject();
    }
}

void FilterOrientationsWidget::onAcceptedDialog()
{
    accept();
}

void FilterOrientationsWidget::onRejectedDialog()
{
    restoreLastIndexes();
    reject();
}
