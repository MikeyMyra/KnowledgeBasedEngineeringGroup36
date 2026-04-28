from parapy.gui import display

if __name__ == '__main__':
    
    from test_aircraft import Aircraft
    ac = Aircraft(label="Aircraft_with_Q3D",
                fu_radius=2.5,
                #fu_sections=[10, 90, 100, 100, 100, 100, 100, 100, 95, 70, 10],
                fu_length=50.65,
                airfoil_root_name="whitcomb",
                airfoil_tip_name="simm_airfoil",
                w_c_root=6., w_c_tip=2.3,
                t_factor_root=1, t_factor_tip=1,
                w_semi_span=27.,
                w_sweep=20, w_twist=-5, w_dihedral=3,
                wing_position_fraction_long=0.4,
                wing_position_fraction_vrt=0.8,
                vt_long=0.8, vt_taper=0.4)
    display(ac)