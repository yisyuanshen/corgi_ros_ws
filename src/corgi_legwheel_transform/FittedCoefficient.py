import numpy as np

#### Fitted Coefficients (all are left side) ####
# x**0 + x**1 + ... + x**7
A_x_coef = [1.1298507215688325e-05, -0.08009406847959913, 0.00031078052601928794, 0.012797805183985959, 0.0005296240879456367, -0.0009747108437109868, 0.00010114657513265072, 3.9686108833838617e-07]
A_y_coef = [0.07999814821731825, 1.7151920935796158e-05, -0.040064720911549265, 0.00013130348527951678, 0.003174527009668425, 0.00011925178438898585, -0.00016662716127708877, 1.5155513490796227e-05]
B_x_coef = [1.4123134019512839e-05, -0.10011758559949738, 0.00038847565752584784, 0.01599725647995614, 0.0006620301099677044, -0.0012183885546565188, 0.0001264332189193294, 4.960763602278808e-07]
B_y_coef = [0.09999768527164826, 2.1439901177591913e-05, -0.050080901139490396, 0.00016412935670033434, 0.003968158761998819, 0.00014906473052470537, -0.0002082839516049707, 1.894439186426742e-05]
C_x_coef = [-0.08730529149658174, -0.006032928410248646, 0.001934196220435525, 0.02955987169250997, -0.022406753082320346, 0.0054248375192732365, 0.0001056921355505886, -0.00014747825272172453]
C_y_coef = [-0.029355119301363412, -0.048032800132052085, -0.050704510018537124, 0.04823978278167183, -0.02452118688980307, 0.0034280732073878437, 0.0009713086749803987, -0.00022015228841656563]
D_x_coef = [3.4764637586991e-06, -0.024644328762954037, 9.562477724102212e-05, 0.003937786210445927, 0.000162961257841487, -0.0002999110288402457, 3.112202311918032e-05, 1.221111039772743e-07]
D_y_coef = [-0.011598147648429716, 0.012042317088583223, -0.057280767044083254, 0.04730088627739169, -0.033523140947929304, 0.010133092141035393, -0.0007682203747394439, -6.530417644880412e-05]
E_y_coef = [-0.05230761247765039, 0.017386834940865032, -0.06493234310296639, 0.06826514529607226, -0.049833215595717455, 0.014583687855084628, -0.0010355951362746368, -0.00010106403864448877]
F_x_coef = [-0.07922197995394527, 0.006429966727950573, 0.002120211801039433, 0.025143170481931387, -0.02091333510279192, 0.005551135130921033, -2.9108620785113164e-05, -0.00013153304571884635]
F_y_coef = [-0.04889045741031588, -0.04099768033007372, -0.050576527465387176, 0.05336639156632884, -0.029257201539865652, 0.004423676195682945, 0.0010571619431373274, -0.00025474474248363046]
G_y_coef = [-0.08004472811678946, -0.04301096555457295, -0.10580886132752444, 0.0888545682810313, -0.031030861225472762, -0.0011104867548842852, 0.0030345590247493667, -0.00046519990417785516]
H_x_coef = [0.02986810945369848, -0.10150955831419921, 0.00033837074403774705, 0.006765373774739306, 0.007615792562309228, -0.0024121994732995045, -6.520446163389534e-05, 5.435366353719506e-05]
H_y_coef = [0.0984219968319075, 0.020210699222679533, -0.04976557612069304, -0.0023520295662631807, 0.0029910867610955538, 0.0009163508036696394, -0.00032806661645546487, 1.823661837557865e-05]
U_x_coef = [0.009669527527254635, -0.03326882772877039, 0.0014183676837420903, 0.002963308891146737, 0.0008700406282839998, 0.0007517214838673226, -0.0003701278840337812, 2.5056958461185766e-05]
U_y_coef = [-0.0006690023490031166, 0.014773023018696044, -0.04975560872784549, 0.029792034672425482, -0.019784732744835352, 0.004526695454915333, 0.0003729635468404478, -0.00016159426009107307]
L_x_coef = [-0.006205715410243533, 0.005373735412447777, 0.06028316700203501, -0.025480735039307013, -0.00855485481636677, 0.008709592938898975, -0.0021348251734653874, 0.00015989475249695854]
L_y_coef = [0.020478449370300727, -0.04889887701569285, -0.08046609883658265, 0.04415062837261041, -0.0077196354531666005, -0.004295629132118317, 0.0020770723033178957, -0.00021893555992257824]
inv_G_dist_coef = [-5.959032367753028, 198.86230658069783, -2844.1971931563417, 23374.510431967738, -113385.41639339655, 325135.5816168529, -511747.5158229085, 342039.91290943703]
inv_U_dist_coef = [0.29524047232829054, 31.242020957998935, -211.52801793007131, -399.42392065317296, 27998.240942773482, -261545.9283385091, 1067954.8094097893, -1657655.6365357146]
inv_L_dist_coef = [0.2953054355683213, 11.024121222923652, -72.41460853902848, 986.7028594015169, -8170.968258339065, 38941.91061949656, -98163.67750980039, 101569.08624570252]


#### Polynomial ####
A_l_poly = np.array([ np.polynomial.Polynomial(A_x_coef), np.polynomial.Polynomial(A_y_coef)])
A_r_poly = np.array([-np.polynomial.Polynomial(A_x_coef), np.polynomial.Polynomial(A_y_coef)])
B_l_poly = np.array([ np.polynomial.Polynomial(B_x_coef), np.polynomial.Polynomial(B_y_coef)])
B_r_poly = np.array([-np.polynomial.Polynomial(B_x_coef), np.polynomial.Polynomial(B_y_coef)])
C_l_poly = np.array([ np.polynomial.Polynomial(C_x_coef), np.polynomial.Polynomial(C_y_coef)])
C_r_poly = np.array([-np.polynomial.Polynomial(C_x_coef), np.polynomial.Polynomial(C_y_coef)])
D_l_poly = np.array([ np.polynomial.Polynomial(D_x_coef), np.polynomial.Polynomial(D_y_coef)])
D_r_poly = np.array([-np.polynomial.Polynomial(D_x_coef), np.polynomial.Polynomial(D_y_coef)])
E_poly   = np.array([ np.polynomial.Polynomial([0])     , np.polynomial.Polynomial(E_y_coef)])
F_l_poly = np.array([ np.polynomial.Polynomial(F_x_coef), np.polynomial.Polynomial(F_y_coef)])
F_r_poly = np.array([-np.polynomial.Polynomial(F_x_coef), np.polynomial.Polynomial(F_y_coef)])
G_poly   = np.array([ np.polynomial.Polynomial([0])     , np.polynomial.Polynomial(G_y_coef)])
H_l_poly = np.array([ np.polynomial.Polynomial(H_x_coef), np.polynomial.Polynomial(H_y_coef)])
H_r_poly = np.array([-np.polynomial.Polynomial(H_x_coef), np.polynomial.Polynomial(H_y_coef)])
U_l_poly = np.array([ np.polynomial.Polynomial(U_x_coef), np.polynomial.Polynomial(U_y_coef)])
U_r_poly = np.array([-np.polynomial.Polynomial(U_x_coef), np.polynomial.Polynomial(U_y_coef)])
L_l_poly = np.array([ np.polynomial.Polynomial(L_x_coef), np.polynomial.Polynomial(L_y_coef)])
L_r_poly = np.array([-np.polynomial.Polynomial(L_x_coef), np.polynomial.Polynomial(L_y_coef)])
inv_G_dist_poly = np.polynomial.Polynomial(inv_G_dist_coef)
inv_U_dist_poly = np.polynomial.Polynomial(inv_U_dist_coef)
inv_L_dist_poly = np.polynomial.Polynomial(inv_L_dist_coef)

#### Derivative ####
A_l_poly_deriv = np.array([poly.deriv() for poly in A_l_poly])
A_r_poly_deriv = np.array([poly.deriv() for poly in A_r_poly])
B_l_poly_deriv = np.array([poly.deriv() for poly in B_l_poly])
B_r_poly_deriv = np.array([poly.deriv() for poly in B_r_poly])
C_l_poly_deriv = np.array([poly.deriv() for poly in C_l_poly])
C_r_poly_deriv = np.array([poly.deriv() for poly in C_r_poly])
D_l_poly_deriv = np.array([poly.deriv() for poly in D_l_poly])
D_r_poly_deriv = np.array([poly.deriv() for poly in D_r_poly])
E_poly_deriv   = np.array([poly.deriv() for poly in E_poly  ])
F_l_poly_deriv = np.array([poly.deriv() for poly in F_l_poly])
F_r_poly_deriv = np.array([poly.deriv() for poly in F_r_poly])
G_poly_deriv   = np.array([poly.deriv() for poly in G_poly  ])
H_l_poly_deriv = np.array([poly.deriv() for poly in H_l_poly])
H_r_poly_deriv = np.array([poly.deriv() for poly in H_r_poly])
U_l_poly_deriv = np.array([poly.deriv() for poly in U_l_poly])
U_r_poly_deriv = np.array([poly.deriv() for poly in U_r_poly])
L_l_poly_deriv = np.array([poly.deriv() for poly in L_l_poly])
L_r_poly_deriv = np.array([poly.deriv() for poly in L_r_poly])
inv_G_dist_poly_deriv = inv_G_dist_poly.deriv()
inv_U_dist_poly_deriv = inv_U_dist_poly.deriv()
inv_L_dist_poly_deriv = inv_L_dist_poly.deriv()