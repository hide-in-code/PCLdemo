#include <vtkRenderWindow.h>
#include <vtkJPEGReader.h>
#include <vtkImageData.h>
#include <vtkLogoRepresentation.h>
#include <vtkLogoWidget.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkScalarBarWidget.h>
#include <vtkScalarBarActor.h>
#include <vtkScalarBarRepresentation.h>
#include <vtkColorTransferFunction.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

using namespace std;

class YcPCLVisualizer :public pcl::visualization::PCLVisualizer
{

	public: 
		YcPCLVisualizer(string name, bool initIterator) :pcl::visualization::PCLVisualizer(name, initIterator)
		{
			;
		}

		/** \brief Adds a widget which shows an Logo image display
		*  \param[in] interactor - Pointer to the vtk interactor object used by the PCLVisualizer window
		*  \param[in] name - imagefilename
		*  \param[in] x - x-position
		*  \param[in] y - y-position
		*  \param[in] x_wide - x-scalar
		*  \param[in] y_wide - y-scalar
		*  \param[in] opaticy - opaticy
		*/
		void addLogoWidgetToview(vtkRenderWindowInteractor* interactor, const std::string &name, double &x, double &y, double &x_wide, double &y_wide, double &opaticy);

	private:

		/** \brief Internal pointer to widget which contains a logo_Widget_ */
		vtkSmartPointer<vtkLogoWidget> logo_Widget_member_;

		/** \brief Internal pointer to widget which contains a scalarbar_Widget_ */
		vtkSmartPointer<vtkScalarBarWidget> scalarbar_Widget_member_;

		/** \brief Internal pointer to widget which contains a set of axes */
		vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget_member_;

};

void YcPCLVisualizer::addLogoWidgetToview(vtkRenderWindowInteractor* interactor, const std::string &name, double &x, double &y, double &x_wide, double &y_wide, double &opaticy)
{
	if (!logo_Widget_member_)
	{
		vtkSmartPointer<vtkJPEGReader> reader = vtkSmartPointer<vtkJPEGReader>::New();
		reader->SetFileName(name.c_str());/*no multiple coding format*/
		reader->Update();
		vtkSmartPointer<vtkLogoRepresentation> logoRepresentation = vtkSmartPointer<vtkLogoRepresentation>::New();
		logoRepresentation->SetImage(reader->GetOutput());
		logoRepresentation->SetPosition(x, y);
		logoRepresentation->SetPosition2(x_wide, y_wide);/*0.12, 0.05*/
		logoRepresentation->GetImageProperty()->SetOpacity(opaticy);/*0.9*/

		logoRepresentation->SetShowBorderToOff();/*no edgeline*/
		logo_Widget_member_ = vtkSmartPointer<vtkLogoWidget>::New();
		logo_Widget_member_->SetInteractor(interactor);
		logo_Widget_member_->SetRepresentation(logoRepresentation);

		logo_Widget_member_->SetEnabled(true);/*enable show*/
		logo_Widget_member_->ProcessEventsOff();/*disable move*/

	}
	else
	{
		logo_Widget_member_->SetEnabled(true);
		pcl::console::print_warn(stderr, "LogoWidget Widget already exists, just enabling it");
	}
}