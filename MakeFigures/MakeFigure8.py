
import os
import numpy as np
import simpleimageio as sio
import figuregen

from typing import Tuple, List
import figuregen.util.image as image
from figuregen.util.image import Cropbox

prefix = "../ControlledMixtureSampling/bin/Release/net7.0/Results/PT/EqualTime"
methods = ["Reference","Baseline","Vevoda et al.","Ours"]

class CropComparison:

    def __init__(self, reference_image, method_images, crops: List[image.Cropbox],
                depth_str, idx = 0, scene_name = None, method_names = None, use_latex = False):
                 
        self._reference_image = reference_image
        self._method_images = method_images
        self.use_latex = use_latex

        self._errors = [
            self.compute_error(reference_image, m)
            for m in method_images
        ]
        self._crop_errors = [
            [
                self.compute_error(crop.crop(reference_image), crop.crop(m))
                for m in method_images
            ]
            for crop in crops
        ]

        # Create the grid for the reference image
        self._ref_grid = figuregen.Grid(1, 1)
        self._ref_grid[0, 0].image = self.tonemap(reference_image)
        for crop in crops:
            self._ref_grid[0, 0].set_marker(crop.marker_pos, crop.marker_size, color=[255,255,255])
            
        if idx==0:
            self._ref_grid.set_col_titles("top", ["GI, Max depth 5"])
        self._ref_grid.set_col_titles("bottom", [scene_name])

        # Create the grid with the crops
        self._crop_grid = figuregen.Grid(num_cols=len(method_images) + 1, num_rows=len(crops))
        for row in range(len(crops)):
            self._crop_grid[row, 0].image = self.tonemap(crops[row].crop(reference_image))
            self._crop_grid[row, 0].set_label(f"{self.error_metric_name} zoom-in",pos="bottom",txt_color=(255,255,255),fontsize=8,width_mm=30,height_mm=5,offset_mm=[0.0,-3.5])
            for col in range(len(method_images)):
                self._crop_grid[row, col + 1].image = self.tonemap(crops[row].crop(method_images[col]))
                self._crop_grid[row, col + 1].set_label(self.error_string(col, self.crop_errors[row]),
                                                        pos="bottom",fontsize=8,txt_color=(255,255,255),width_mm=20,height_mm=5,offset_mm=[0.0,-3.5])
       # Put error values underneath the columns
        error_strings = [ f"{self.error_metric_name} full image" ]
        error_strings.extend([ self.error_string(i, self.errors) for i in range(len(self.errors)) ])
        self._crop_grid.set_col_titles("bottom", error_strings)
        self._crop_grid.layout.set_padding(column=1, row=1)
        self._crop_grid.layout.set_col_titles("bottom", fontsize=8, field_size_mm=5.5, offset_mm=0.5)

        # If given, show method names on top
        if method_names is not None and idx ==0:
            self._crop_grid.set_col_titles("top", method_names)
            self._crop_grid.layout.set_col_titles("top", fontsize=8, field_size_mm=2.8, offset_mm=0.25)

        self._ref_grid.copy_layout(self._crop_grid)
        self._ref_grid.layout.set_padding(right=1)

    def tonemap(self, img):
        return figuregen.PNG(image.lin_to_srgb(img))

    @property
    def error_metric_name(self) -> str:
        return "relMSE"

    def compute_error(self, reference_image, method_image) -> Tuple[str, List[float]]:
        return image.relative_mse_outlier_rejection(method_image, reference_image)

    def error_string(self, index: int, errors: List[float]):
        """ Generates the human-readable error string for the i-th element in a list of error values.

        Args:
            index: index in the list of errors
            errors: list of error values, one per method, in order
        """
        if self.use_latex and index == np.argmin(errors):
            return f"$\\mathbf{{{errors[index]:.2e} ({errors[0]/errors[index]:.2f}\\times)}}$"
        elif self.use_latex:
            return f"${errors[index]:.2e} ({errors[0]/errors[index]:.2f}\\times)$"
        else:
            return f"{errors[index]:.2e} ({errors[0]/errors[index]:.2f}x)"
            #return f"{croped_errors[index]:.2f} ({croped_errors[0]/croped_errors[index]:.2f}x)\\\\{errors[index]:.2f} ({errors[0]/errors[index]:.2f}x)"

    @property
    def crop_errors(self) -> List[List[float]]:
        """ Error values within the cropped region of each method.
        First dimension is the crop, second the method.
        """
        return self._crop_errors

    @property
    def errors(self) -> List[float]:
        return self._errors

    @property
    def figure_row(self) -> List[figuregen.Grid]:
        return [ self._ref_grid, self._crop_grid ]

scenes = [
    ("DiningRoom2",
     "Dining Room",
     "32",
     [
       Cropbox(top=150, left=80, height=96, width=128, scale=5),
    ]), 
    ("RGBSofa",
     "RGB Sofa",
     "64",
     [
       Cropbox(top=60, left=430, height=96, width=128, scale=5),
    ]), 
    ("BathRoom",
     "Bathroom",
     "64",
     [
       Cropbox(top=300, left=300, height=96, width=128, scale=5),
    ]), 

]

rows = []
i = 0
for s, display_name,grid, crop in scenes:
    ref_folder = os.path.join(prefix, "{}/Reference.exr".format(s))    
    figure = CropComparison(
                reference_image=sio.read(ref_folder),
                method_images=[
                    sio.read(os.path.join(prefix ,s,methods[1], "Render.exr")),
                    sio.read(os.path.join(prefix ,s,grid,methods[2], "Render.exr")),
                    sio.read(os.path.join(prefix ,s,grid,methods[3], "Render.exr")),
                    ],
                    crops=crop,
                    scene_name = display_name,
                    depth_str = "",
                    method_names = methods,
                    idx = i
            )

    rows.append(figure.figure_row)
    i+=1
            
if not os.path.exists(os.path.join("Fig.8")):
    os.makedirs(os.path.join("Fig.8"))
            
figuregen.figure(rows, 17.8, os.path.join('Fig.8', "Fig.8.pdf"), figuregen.PdfBackend(None, [
            "\\usepackage[utf8]{inputenc}",
            "\\usepackage[T1]{fontenc}",
            "\\usepackage{libertine}",
            "\\usepackage{color}",
            "\\usepackage{xparse}",
            "\\usepackage[outline]{contour}",
            "\\renewcommand{\\familydefault}{\\sfdefault}",
        ]))            

