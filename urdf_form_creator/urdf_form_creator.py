import sys
import os
from tkinter import Tk, StringVar
from tkinter import Grid, Label, OptionMenu, Entry, Button

from lxml import etree

changable_label_1: Label
changable_label_2: Label
changable_entry_1: Entry
changable_entry_2: Entry
selected_geometry_option: StringVar

def callback(root: Tk, selected_geometry_option: str, *args):
    global changable_label_1
    global changable_label_2
    global changable_entry_1
    global changable_entry_2

    changable_entry_1 = Entry(root, width=40)
    changable_entry_2 = Entry(root, width=40)

    changable_entry_1.grid(row= 1, column= 1, padx= 5, pady= 5)

    if selected_geometry_option == 'Box':
        changable_label_1 = Label(root, text="Width: ", borderwidth=10)
        changable_label_2 = Label(root, text="Length: ", borderwidth=10)

        changable_label_1.grid(row = 1, column= 0)
        changable_label_2.grid(row = 2, column= 0)

        changable_entry_2.grid(row= 2, column= 1, padx= 5, pady= 5)
    elif selected_geometry_option == 'Cylinder':
        changable_label_1 = Label(root, text="Radius: ", borderwidth=10)
        changable_label_2 = Label(root, text="        ", borderwidth=10) #Empty line

        changable_label_1.grid(row= 1, column= 0)
        changable_label_2.grid(row= 2, column= 0)

        changable_entry_2.grid_forget()
        temp: Label = Label(root, text="", width=40)
        temp.grid(row=2, column=1)
    elif selected_geometry_option == 'Isosceles Triangle':
        changable_label_1 = Label(root, text="Scale: ", borderwidth=10)
        changable_label_2 = Label(root, text="        ", borderwidth=10) #Empty line

        changable_label_1.grid(row= 1, column= 0)
        changable_label_2.grid(row= 2, column= 0)

        changable_entry_2.grid_forget()
        temp: Label = Label(root, text="", width=40)
        temp.grid(row=2, column=1)
    elif selected_geometry_option == 'Right Angled Triangle':
        changable_label_1 = Label(root, text="Scale X: ", borderwidth=10)
        changable_label_2 = Label(root, text="Scale Y: ", borderwidth=10)

        changable_label_1.grid(row= 1, column= 0)
        changable_label_2.grid(row= 2, column= 0)

        changable_entry_2.grid(row= 2, column= 1, padx= 5, pady= 5)

def on_click_create_urdf():
    global changable_entry_1
    global changable_entry_2
    global selected_geometry_option

    value_1: float= float(changable_entry_1.get())
    value_2: float
    if selected_geometry_option.get() != "Cylinder" and selected_geometry_option.get() != "Isosceles Triangle":
        value_2= float(changable_entry_2.get())

    root_link_name:str = "object_link"

    robot = etree.Element("robot", name="object")
    root_link_element = etree.SubElement(robot, "link", name=root_link_name)
    visual_element= etree.SubElement(root_link_element, "visual")

    geometry_element= etree.SubElement(visual_element, "geometry")
    if selected_geometry_option.get() == "Box":
        etree.SubElement(geometry_element, "box", size=str(value_1) + " " + str(value_2) + " 0.1")
    elif selected_geometry_option.get() == "Cylinder":
        etree.SubElement(geometry_element, "cylinder", radius=str(value_1), length=str(0.1))
    elif selected_geometry_option.get() == "Isosceles Triangle":
        etree.SubElement(geometry_element, "mesh", filename="package://urdf_form_creator/meshes/Isosceles_Triangle.stl", scale=str(value_1 * 0.001) + " " + str(value_1 * 0.001) + " 0.001")
    elif selected_geometry_option.get() == "Right Angled Triangle":
        etree.SubElement(geometry_element, "mesh", filename="package://urdf_form_creator/meshes/Right_Angled_Triangle.stl", scale=str(value_1 * 0.001) + " " + str(value_2 * 0.001) + " 0.001")
    etree.SubElement(visual_element, "material", name="white")

    collision_element= etree.SubElement(root_link_element, "collision")
    geometry_element= etree.SubElement(collision_element, "geometry")
    if selected_geometry_option.get() == "Box":
        etree.SubElement(geometry_element, "box", size=str(value_1) + " " + str(value_2) + " 0.1")
    elif selected_geometry_option.get() == "Cylinder":
        etree.SubElement(geometry_element, "cylinder", radius=str(value_1), length=str(0.1))
    elif selected_geometry_option.get() == "Isosceles Triangle":
        etree.SubElement(geometry_element, "mesh", filename="package://urdf_form_creator/meshes/Isosceles_Triangle.stl", scale=str(value_1 * 0.001) + " " + str(value_1 * 0.001) + " 0.001")
    elif selected_geometry_option.get() == "Right Angled Triangle":
        etree.SubElement(geometry_element, "mesh", filename="package://urdf_form_creator/meshes/Right_Angled_Triangle.stl", scale=str(value_1 * 0.001) + " " + str(value_2 * 0.001) + " 0.001")

    inertial_element= etree.SubElement(root_link_element, "inertial")
    etree.SubElement(inertial_element, "mass", value="1.0")
    etree.SubElement(inertial_element, "inertia", ixx="1.0", iyy="1.0", izz="1.0", ixy="0.0", ixz="0.0", iyz="0.0")


    gazebo_element = etree.SubElement(robot, "gazebo", reference=root_link_name)
    material_element= etree.SubElement(gazebo_element, "material")
    material_element.text= "Gazebo/DarkGrey"
    etree.SubElement(gazebo_element, "mu1", value="100000.0")
    etree.SubElement(gazebo_element, "mu2", value="100000.0")
    etree.SubElement(gazebo_element, "kp", value="10000.0")
    etree.SubElement(gazebo_element, "kd", value="1.0")
    
    tree = etree.ElementTree(robot)

    script_dir: str = os.path.dirname(__file__)
    urdf_file_path: str = os.path.join(script_dir, "urdf/basic_geometry.urdf")
    tree.write(urdf_file_path, encoding="utf-8", xml_declaration=True, pretty_print=True)


def create_gui():
    global selected_geometry_option
    
    root = Tk()
    root.title("URDF Form Creator")

    geometry_options = {'Box', 'Cylinder', 'Isosceles Triangle', 'Right Angled Triangle'}
    selected_geometry_option = StringVar(root)
    selected_geometry_option.trace("w", lambda *args: callback(root, selected_geometry_option.get(), *args))
    selected_geometry_option.set('Box')

    geometry_shape_label = Label(root, text="Geometry Shape:", borderwidth=10)
    geometry_shape_options = OptionMenu(root, selected_geometry_option,*geometry_options)
    geometry_shape_options.config(width=20)

    create_urdf_button: Button = Button(root, text="Create urdf", command=on_click_create_urdf)
    exit_button: Button = Button(root, text="Exit", command=root.quit)

    geometry_shape_label.grid(row=0, column=0)
    geometry_shape_options.grid(row=0, column=1)
    create_urdf_button.grid(row=3,column=0)
    exit_button.grid(row=3,column=1)

    root.mainloop()


if __name__ == '__main__':
    create_gui()