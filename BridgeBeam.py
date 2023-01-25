import NemAll_Python_Geometry as geo
import NemAll_Python_BaseElements as base
import NemAll_Python_BasisElements as basis
import NemAll_Python_Utility as util
import val as val
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties

import StdReinfShapeBuilder.GeneralReinfShapeBuilder as GeneralShapeBuilder
import StdReinfShapeBuilder.LinearBarPlacementBuilder as LinearBarBuilder

from StdReinfShapeBuilder.ConcreteCoverProperties import ConcreteCoverProperties
from StdReinfShapeBuilder.ReinforcementShapeProperties import ReinforcementShapeProperties
from StdReinfShapeBuilder.RotationAngles import RotationAngles
import NemAll_Python_Reinforcement as AllplanReinf

print('Load BridgeBeam.py')

def check_allplan_version(build_part, version):
    del build_part
    del version
    return True

def create_element(build_part, doc):
    element = Armature(doc)
    return element.create(build_part)

def move_handle(build_part, handle_prop, input_pnt, doc):
    build_part.change_property(handle_prop, input_pnt)
    return create_element(build_part, doc)

class Armature:
    def __init__(self, doc):
        self.model_part_list = []
        self.handle_list = []
        self.document = doc

    def create(self, build_part):
        self.top(build_part)
        self.create_handles(build_part)
        return (self.model_part_list, self.handle_list)

    def get(self, build_part):
        self.width_d = build_part.width_d.value
        self.len = build_part.len.value
        self.height_b = build_part.height_b.value
        self.cut_b_t = build_part.cut_b_t.value
        self.cut_b_b = build_part.cut_b_b.value
        self.center_w = build_part.center_w.value
        self.middle_h = build_part.middle_h.value
        self.Radius = build_part.Radius.value
        self.width_t = build_part.width_t.value
        self.height_t = build_part.height_t.value
        self.PlateSpace = build_part.PlateSpace.value
        self.PlateHeight = build_part.PlateHeight.value
        self.color = build_part.color.value
        self.cut_t_t = build_part.cut_t_t.value
        self.Deep = build_part.Deep.value
        self.BarSpacing = build_part.BarSpacing.value

    def reif_create(self, build_part):
        self.get(build_part)
        model_angles = RotationAngles(0, 90, 90)
        steel_grade = AllplanReinf.ReinforcementSettings.GetSteelGrade()
        shape_props = ReinforcementShapeProperties.rebar(25, 4, steel_grade, -1, AllplanReinf.BendingShapeType.LongitudinalBar)
        concrete_cover_props = ConcreteCoverProperties.left_right_bottom( 20. * 2, 20. * 2, 20.)
        shape = GeneralShapeBuilder.create_longitudinal_shape_with_hooks(
            self.Deep, model_angles, shape_props, concrete_cover_props, 0, -1)
        height = self.height_b + self.middle_h + \
                 self.height_t + self.PlateHeight + 200
        self.model_part_list.append(LinearBarBuilder.create_linear_bar_placement_from_to_by_dist(1, shape, geo.Point3D(
            self.width_t / 6, 0, height), geo.Point3D(self.width_t / 6, self.len, height), 0, 0, self.BarSpacing))
        model_angles = RotationAngles(0, 90, 270)
        steel_grade = AllplanReinf.ReinforcementSettings.GetSteelGrade()
        shape_props = ReinforcementShapeProperties.rebar(25, 4, steel_grade, -1, AllplanReinf.BendingShapeType.LongitudinalBar)
        concrete_cover_props = ConcreteCoverProperties.left_right_bottom(20. * 2, 20. * 2, 20.)
        shape = GeneralShapeBuilder.create_longitudinal_shape_with_hooks(self.Deep, model_angles, shape_props, concrete_cover_props, 0, -1)
        self.model_part_list.append(LinearBarBuilder.create_linear_bar_placement_from_to_by_dist(1, shape, geo.Point3D(
            self.width_t - self.width_t / 3, 0, height), geo.Point3D(self.width_t - self.width_t / 3, self.len, height), 0, 0, self.BarSpacing))

    def bottom(self, build_part):
        self.get(build_part)
        f = geo.BRep3D.CreateCuboid(
            geo.AxisPlacement3D(geo.Point3D(0, 0, 0),
                                geo.Vector3D(1, 0, 0),
                                geo.Vector3D(0, 0, 1)),
            self.width_d,
            self.len,
            self.height_b)
        f_i = geo.BRep3D.CreateCuboid(
            geo.AxisPlacement3D(geo.Point3D(0, 0, 0),
                                geo.Vector3D(1, 0, 0),
                                geo.Vector3D(0, 0, 1)),
            self.width_d,
            self.len,
            self.height_b)
        c_w = self.cut_b_t
        c_w_b = self.cut_b_b
        if c_w > 0:
            ed = util.VecSizeTList()
            ed.append(1)
            ed.append(3)
            e, f = geo.ChamferCalculus.Calculate(
                f, ed, c_w, False)
            if not val.polyhedron(e):
                return
        if c_w_b > 0:
            ed2 = util.VecSizeTList()
            ed2.append(8)
            ed2.append(10)
            e, f_i = geo.ChamferCalculus.Calculate(
                f_i, ed2, c_w_b, False)
            if not val.polyhedron(e):
                return
        e, end = geo.MakeIntersection(f, f_i)
        return end

    def middle(self, build_part):
        self.get(build_part)
        f = geo.BRep3D.CreateCuboid(
            geo.AxisPlacement3D(geo.Point3D(
                self.width_d / 2 - self.center_w / 2, 0, self.height_b),
                                geo.Vector3D(1, 0, 0),
                                geo.Vector3D(0, 0, 1)),
            self.center_w,
            self.len,
            self.middle_h)
        ce = geo.BRep3D.CreateCylinder(
            geo.AxisPlacement3D(geo.Point3D(
                self.cut_b_t, self.len / 8, self.height_b + self.middle_h / 2),
                                geo.Vector3D(0, 0, 1),
                                geo.Vector3D(1, 0, 0)),
            self.Radius, self.center_w)
        ce1 = geo.BRep3D.CreateCylinder(
            geo.AxisPlacement3D(geo.Point3D(
                self.cut_b_t, self.len - self.len / 8, self.height_b + self.middle_h / 2),
                                geo.Vector3D(0, 0, 1),
                                geo.Vector3D(1, 0, 0)),
            self.Radius, self.center_w)
        e, f = geo.MakeSubtraction(f, ce)
        e, f = geo.MakeSubtraction(f, ce1)
        e, end = geo.MakeUnion(f, self.bottom(build_part))
        return end

    def top(self, build_part):
        self.get(build_part)
        f = geo.BRep3D.CreateCuboid(geo.AxisPlacement3D(geo.Point3D(0 - (self.width_t - self.width_d) / 2, 0, self.height_b + self.middle_h),
                                       geo.Vector3D(1, 0, 0),
                                       geo.Vector3D(0, 0, 1)),
            self.width_t,
            self.len,
            self.height_t)
        f_p = geo.BRep3D.CreateCuboid(geo.AxisPlacement3D(geo.Point3D(
            self.PlateSpace - (self.width_t - self.width_d) / 2, 0, self.height_b + self.middle_h + self.height_t),
                                       geo.Vector3D(1, 0, 0),
                                        geo.Vector3D(0, 0, 1)),
            self.width_t - self.PlateSpace*2,
            self.len,
            self.PlateHeight)
        com_prop = base.CommonProperties()
        com_prop.GetGlobalProperties()
        com_prop.Pen = 1
        com_prop.Color = self.color
        chamfer_width_top = self.cut_t_t
        if chamfer_width_top > 0:
            ed2 = util.VecSizeTList()
            ed2.append(8)
            ed2.append(10)
            e, f = geo.ChamferCalculus.Calculate(f, ed2, chamfer_width_top, False)
            if not val.polyhedron(e):
                return
        e, end = geo.MakeUnion(f, self.middle(build_part))
        e, end = geo.MakeUnion(end, f_p)
        self.model_part_list.append(
            basis.ModelElement3D(com_prop, end))
        self.reif_create(build_part)

    def create_handles(self, build_part):
        self.get(build_part)
        origin = geo.Point3D(
            self.width_d / 2, self.len, self.middle_h + self.height_b)
        origin2 = geo.Point3D(
            self.width_d / 2, 0, self.height_b / 2)
        origin3 = geo.Point3D(
            0, self.len, (self.height_b - self.cut_b_t) / 2)
        origin4 = geo.Point3D(
            0 - (self.width_t - self.width_d) / 2, self.len, self.middle_h + self.height_b + self.cut_t_t)
        origin5 = geo.Point3D(
            self.width_d / 2, self.len, self.middle_h + self.height_b - self.height_b / 4)
        origin6 = geo.Point3D(
            self.width_d / 2, self.len, self.middle_h + self.height_b + self.height_t)
        origin7 = geo.Point3D(
            self.width_d / 2, self.len, 0)
        origin8 = geo.Point3D(
            self.width_d / 2 - self.center_w / 2, self.len, self.middle_h / 2 + self.height_b)

        self.handle_list.append(HandleProperties("middle_h",
                             geo.Point3D(origin.X,
                                         origin.Y,
                                         origin.Z),

                             geo.Point3D(origin.X,
                                         origin.Y,
                                         origin.Z - self.middle_h),
                             [("middle_h", HandleDirection.z_dir)],
                                            HandleDirection.z_dir,
                             False))
        self.handle_list.append(HandleProperties("len",
                             geo.Point3D(origin2.X,
                                         origin2.Y + self.len,
                                         origin2.Z),
                                         geo.Point3D(origin2.X,
                                         origin2.Y,
                                         origin2.Z),
                             [("len", HandleDirection.y_dir)],
                                    HandleDirection.y_dir,
                             False))
        self.handle_list.append(HandleProperties("width_d", geo.Point3D(origin3.X + self.width_d, origin3.Y, origin3.Z),
                             geo.Point3D(origin3.X, origin3.Y, origin3.Z),
                             [("width_d", HandleDirection.x_dir)],
                                          HandleDirection.x_dir,
                             False))
        self.handle_list.append(HandleProperties("width_t",
                             geo.Point3D(origin4.X + self.width_t,
                                         origin4.Y,
                                         origin4.Z),
                            geo.Point3D(origin4.X,
                                        origin4.Y,
                                        origin4.Z),
                             [("width_t", HandleDirection.x_dir)],
                                            HandleDirection.x_dir,
                             False))
        self.handle_list.append(HandleProperties("height_t",
                             geo.Point3D(origin5.X,
                                         origin5.Y,
                                        origin5.Z + self.height_t),
                             geo.Point3D(origin5.X,
                                                origin5.Y,
                                                origin5.Z),
                             [("height_t", HandleDirection.z_dir)],
                                            HandleDirection.z_dir,
                             False))
        self.handle_list.append(HandleProperties("PlateHeight",
                             geo.Point3D(origin6.X,
                                         origin6.Y,
                                         origin6.Z + self.PlateHeight),
                                         geo.Point3D(origin6.X,
                                         origin6.Y,
                                         origin6.Z),
                                         [("PlateHeight", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))
        self.handle_list.append(
            HandleProperties("height_b",
                             geo.Point3D(origin7.X,
                                         origin7.Y,
                                         origin7.Z + self.height_b),
                             geo.Point3D(origin7.X,
                                         origin7.Y,
                                         origin7.Z),
                             [("height_b", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))
        self.handle_list.append(            HandleProperties("center_w",
                             geo.Point3D(origin8.X + self.center_w,
                                         origin8.Y,
                                         origin8.Z),
                             geo.Point3D(origin8.X,
                                         origin8.Y,
                                         origin8.Z),
                             [("center_w", HandleDirection.x_dir)],
                                           HandleDirection.x_dir,
                             False))