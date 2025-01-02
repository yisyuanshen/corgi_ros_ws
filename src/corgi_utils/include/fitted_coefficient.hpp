#ifndef FITTEDCOEFFICIENT_HPP
#define FITTEDCOEFFICIENT_HPP

#include <vector>
#include <array>

/* Fitted Coefficients (all are left side) */
// x**0 + x**1 + ... + x**7
const std::vector<double> A_x_coef = {1.1298507215688325e-05, -0.08009406847959913, 0.00031078052601928794, 0.012797805183985959, 0.0005296240879456367, -0.0009747108437109868, 0.00010114657513265072, 3.9686108833838617e-07};
const std::vector<double> A_y_coef = {0.07999814821731825, 1.7151920935796158e-05, -0.040064720911549265, 0.00013130348527951678, 0.003174527009668425, 0.00011925178438898585, -0.00016662716127708877, 1.5155513490796227e-05};
const std::vector<double> B_x_coef = {1.4123134019512839e-05, -0.10011758559949738, 0.00038847565752584784, 0.01599725647995614, 0.0006620301099677044, -0.0012183885546565188, 0.0001264332189193294, 4.960763602278808e-07};
const std::vector<double> B_y_coef = {0.09999768527164826, 2.1439901177591913e-05, -0.050080901139490396, 0.00016412935670033434, 0.003968158761998819, 0.00014906473052470537, -0.0002082839516049707, 1.894439186426742e-05};
const std::vector<double> C_x_coef = {-0.08730529149658174, -0.006032928410248646, 0.001934196220435525, 0.02955987169250997, -0.022406753082320346, 0.0054248375192732365, 0.0001056921355505886, -0.00014747825272172453};
const std::vector<double> C_y_coef = {-0.029355119301363412, -0.048032800132052085, -0.050704510018537124, 0.04823978278167183, -0.02452118688980307, 0.0034280732073878437, 0.0009713086749803987, -0.00022015228841656563};
const std::vector<double> D_x_coef = {3.4764637586991e-06, -0.024644328762954037, 9.562477724102212e-05, 0.003937786210445927, 0.000162961257841487, -0.0002999110288402457, 3.112202311918032e-05, 1.221111039772743e-07};
const std::vector<double> D_y_coef = {-0.011598147648429716, 0.012042317088583223, -0.057280767044083254, 0.04730088627739169, -0.033523140947929304, 0.010133092141035393, -0.0007682203747394439, -6.530417644880412e-05};
const std::vector<double> E_y_coef = {-0.05230761247765039, 0.017386834940865032, -0.06493234310296639, 0.06826514529607226, -0.049833215595717455, 0.014583687855084628, -0.0010355951362746368, -0.00010106403864448877};
const std::vector<double> F_x_coef = {-0.07922197995394527, 0.006429966727950573, 0.002120211801039433, 0.025143170481931387, -0.02091333510279192, 0.005551135130921033, -2.9108620785113164e-05, -0.00013153304571884635};
const std::vector<double> F_y_coef = {-0.04889045741031588, -0.04099768033007372, -0.050576527465387176, 0.05336639156632884, -0.029257201539865652, 0.004423676195682945, 0.0010571619431373274, -0.00025474474248363046};
const std::vector<double> G_y_coef = {-0.08004472811678946, -0.04301096555457295, -0.10580886132752444, 0.0888545682810313, -0.031030861225472762, -0.0011104867548842852, 0.0030345590247493667, -0.00046519990417785516};
const std::vector<double> H_x_coef = {0.02986810945369848, -0.10150955831419921, 0.00033837074403774705, 0.006765373774739306, 0.007615792562309228, -0.0024121994732995045, -6.520446163389534e-05, 5.435366353719506e-05};
const std::vector<double> H_y_coef = {0.0984219968319075, 0.020210699222679533, -0.04976557612069304, -0.0023520295662631807, 0.0029910867610955538, 0.0009163508036696394, -0.00032806661645546487, 1.823661837557865e-05};
const std::vector<double> U_x_coef = {0.009669527527254635, -0.03326882772877039, 0.0014183676837420903, 0.002963308891146737, 0.0008700406282839998, 0.0007517214838673226, -0.0003701278840337812, 2.5056958461185766e-05};
const std::vector<double> U_y_coef = {-0.0006690023490031166, 0.014773023018696044, -0.04975560872784549, 0.029792034672425482, -0.019784732744835352, 0.004526695454915333, 0.0003729635468404478, -0.00016159426009107307};
const std::vector<double> L_x_coef = {-0.006205715410243533, 0.005373735412447777, 0.06028316700203501, -0.025480735039307013, -0.00855485481636677, 0.008709592938898975, -0.0021348251734653874, 0.00015989475249695854};
const std::vector<double> L_y_coef = {0.020478449370300727, -0.04889887701569285, -0.08046609883658265, 0.04415062837261041, -0.0077196354531666005, -0.004295629132118317, 0.0020770723033178957, -0.00021893555992257824};
const std::vector<double> inv_G_dist_coef = {-5.959032367753028, 198.86230658069783, -2844.1971931563417, 23374.510431967738, -113385.41639339655, 325135.5816168529, -511747.5158229085, 342039.91290943703};
const std::vector<double> inv_U_dist_coef = {0.29524047232829054, 31.242020957998935, -211.52801793007131, -399.42392065317296, 27998.240942773482, -261545.9283385091, 1067954.8094097893, -1657655.6365357146};
const std::vector<double> inv_L_dist_coef = {0.2953054355683213, 11.024121222923652, -72.41460853902848, 986.7028594015169, -8170.968258339065, 38941.91061949656, -98163.67750980039, 101569.08624570252};


/* polynomial class */
class polynomial {
    public:
        std::vector<double> coef; // Coefficients

        polynomial(const std::vector<double>& c) : coef(c) {}

        // Evaluate the polynomial at x
        double operator()(double x) const {
            double result = 0.0;
            double power = 1.0;
            for (double c : coef) {
                result += c * power;
                power *= x;
            }
            return result;
        }

        // Unary minus operator
        polynomial operator-() const {
            std::vector<double> neg_coef(coef.size());
            for (size_t i = 0; i < coef.size(); ++i)
                neg_coef[i] = -coef[i];
            return polynomial(neg_coef);
        }

        // Compute the derivative polynomial
        polynomial derivative() const {
            if (coef.size() <= 1)
                return polynomial({0}); // Derivative of constant is zero
            std::vector<double> deriv_coef(coef.size() - 1);
            for (size_t i = 1; i < coef.size(); ++i) {
                deriv_coef[i - 1] = coef[i] * i;
            }
            return polynomial(deriv_coef);
        }
};

/* Polynomial */
const std::array<polynomial, 2> A_l_poly = { polynomial(A_x_coef), polynomial(A_y_coef)};
const std::array<polynomial, 2> A_r_poly = {-polynomial(A_x_coef), polynomial(A_y_coef)};
const std::array<polynomial, 2> B_l_poly = { polynomial(B_x_coef), polynomial(B_y_coef)};
const std::array<polynomial, 2> B_r_poly = {-polynomial(B_x_coef), polynomial(B_y_coef)};
const std::array<polynomial, 2> C_l_poly = { polynomial(C_x_coef), polynomial(C_y_coef)};
const std::array<polynomial, 2> C_r_poly = {-polynomial(C_x_coef), polynomial(C_y_coef)};
const std::array<polynomial, 2> D_l_poly = { polynomial(D_x_coef), polynomial(D_y_coef)};
const std::array<polynomial, 2> D_r_poly = {-polynomial(D_x_coef), polynomial(D_y_coef)};
const std::array<polynomial, 2> E_poly   = { polynomial({0})     , polynomial(E_y_coef)};
const std::array<polynomial, 2> F_l_poly = { polynomial(F_x_coef), polynomial(F_y_coef)};
const std::array<polynomial, 2> F_r_poly = {-polynomial(F_x_coef), polynomial(F_y_coef)};
const std::array<polynomial, 2> G_poly   = { polynomial({0})     , polynomial(G_y_coef)};
const std::array<polynomial, 2> H_l_poly = { polynomial(H_x_coef), polynomial(H_y_coef)};
const std::array<polynomial, 2> H_r_poly = {-polynomial(H_x_coef), polynomial(H_y_coef)};
const std::array<polynomial, 2> U_l_poly = { polynomial(U_x_coef), polynomial(U_y_coef)};
const std::array<polynomial, 2> U_r_poly = {-polynomial(U_x_coef), polynomial(U_y_coef)};
const std::array<polynomial, 2> L_l_poly = { polynomial(L_x_coef), polynomial(L_y_coef)};
const std::array<polynomial, 2> L_r_poly = {-polynomial(L_x_coef), polynomial(L_y_coef)};
const polynomial inv_G_dist_poly(inv_G_dist_coef);
const polynomial inv_U_dist_poly(inv_U_dist_coef);
const polynomial inv_L_dist_poly(inv_L_dist_coef);

/* Derivative */
const std::array<polynomial, 2> A_l_poly_deriv = {A_l_poly[0].derivative(), A_l_poly[1].derivative()};
const std::array<polynomial, 2> A_r_poly_deriv = {A_r_poly[0].derivative(), A_r_poly[1].derivative()};
const std::array<polynomial, 2> B_l_poly_deriv = {B_l_poly[0].derivative(), B_l_poly[1].derivative()};
const std::array<polynomial, 2> B_r_poly_deriv = {B_r_poly[0].derivative(), B_r_poly[1].derivative()};
const std::array<polynomial, 2> C_l_poly_deriv = {C_l_poly[0].derivative(), C_l_poly[1].derivative()};
const std::array<polynomial, 2> C_r_poly_deriv = {C_r_poly[0].derivative(), C_r_poly[1].derivative()};
const std::array<polynomial, 2> D_l_poly_deriv = {D_l_poly[0].derivative(), D_l_poly[1].derivative()};
const std::array<polynomial, 2> D_r_poly_deriv = {D_r_poly[0].derivative(), D_r_poly[1].derivative()};
const std::array<polynomial, 2> E_poly_deriv   = {E_poly[0].derivative()  , E_poly[1].derivative()  };
const std::array<polynomial, 2> F_l_poly_deriv = {F_l_poly[0].derivative(), F_l_poly[1].derivative()};
const std::array<polynomial, 2> F_r_poly_deriv = {F_r_poly[0].derivative(), F_r_poly[1].derivative()};
const std::array<polynomial, 2> G_poly_deriv   = {G_poly[0].derivative()  , G_poly[1].derivative()  };
const std::array<polynomial, 2> H_l_poly_deriv = {H_l_poly[0].derivative(), H_l_poly[1].derivative()};
const std::array<polynomial, 2> H_r_poly_deriv = {H_r_poly[0].derivative(), H_r_poly[1].derivative()};
const std::array<polynomial, 2> U_l_poly_deriv = {U_l_poly[0].derivative(), U_l_poly[1].derivative()};
const std::array<polynomial, 2> U_r_poly_deriv = {U_r_poly[0].derivative(), U_r_poly[1].derivative()};
const std::array<polynomial, 2> L_l_poly_deriv = {L_l_poly[0].derivative(), L_l_poly[1].derivative()};
const std::array<polynomial, 2> L_r_poly_deriv = {L_r_poly[0].derivative(), L_r_poly[1].derivative()};
const polynomial inv_G_dist_poly_deriv = inv_G_dist_poly.derivative();
const polynomial inv_U_dist_poly_deriv = inv_U_dist_poly.derivative();
const polynomial inv_L_dist_poly_deriv = inv_L_dist_poly.derivative();

#endif // FITTEDCOEFFICIENT_HPP