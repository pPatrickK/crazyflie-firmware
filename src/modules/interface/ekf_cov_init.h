float ekf_cov_init[15][15] = {
	{ 0.016580786,0.0121999344,-0.00145880889,0.0211111797,0.0074275678,3.78014399e-05,0.00117146979,-0.00116901581,0.000103349777,-3.8133091e-06,1.55429375e-05,-4.25227015e-06,-0.000344432741,-0.000188322508,-3.79893006e-06,},
	{ 0.0129065971,0.0508419022,-0.00197389784,0.0179284871,0.0431547927,0.000622902346,0.00203193834,0.000401913571,-0.000231214342,-1.65915236e-05,1.14313417e-05,7.93242687e-06,0.000311267088,-0.000201092427,4.83875944e-06,},
	{ -0.00134547756,-0.000886479514,0.0141715508,-0.00272015007,0.00567309807,0.00793510543,0.000687618073,0.000684952052,2.20003551e-05,-8.60830076e-06,-7.99656033e-07,1.10761027e-06,-0.000106383033,-0.000356814673,-6.87630098e-05,},
	{ 0.0209634367,0.0162415659,-0.00260662288,0.0436959448,0.00828252369,-0.00165611784,0.00163840258,-0.00206000698,-0.00136299259,-1.33152712e-06,3.20329148e-05,4.13496124e-06,0.000341541553,-0.000100600014,2.5055558e-05,},
	{ 0.00931492288,0.0460597807,0.00356502459,0.0152621164,0.0650352193,-0.00163535375,0.00249207619,0.00125553863,-3.48863386e-05,-2.96721382e-05,6.69571914e-06,6.77958463e-06,0.000273857319,0.000241559076,2.6819563e-05,},
	{ -2.90257427e-05,0.00053503719,0.00795878288,-0.00187129832,-0.00208383276,0.0129831705,0.000132746917,8.34836503e-05,2.01402889e-05,-1.28098761e-06,8.38029756e-07,-2.32386376e-08,-0.000309256651,9.42507698e-05,-0.000143135503,},
	{ 0.0012379157,0.00244175438,0.000642141529,0.00171430383,0.00365244546,0.0001330219,0.00049196433,2.91327084e-05,5.45710293e-05,-3.53179766e-06,2.10830856e-06,-6.55503604e-07,-3.62213013e-05,-8.04043903e-05,-2.01118492e-06,},
	{ -0.00112921093,0.000810737713,0.000687013243,-0.00232056505,0.00192342392,8.35057584e-05,4.59062114e-05,0.000464144925,-7.41741517e-05,-1.59343339e-06,-2.82014814e-06,1.99945626e-06,6.82563701e-05,-5.01589741e-05,-2.28078959e-07,},
	{ 0.000118011626,-0.000151939329,-3.89530225e-06,-0.00137090946,5.09124244e-05,1.44522817e-05,4.85671514e-05,-7.7773341e-05,0.000550829253,-1.49998363e-06,-1.78522436e-06,-5.36453749e-06,3.66012735e-05,3.38432542e-06,-5.35444414e-07,},
	{ -5.27040186e-06,-2.18148538e-05,-1.03669872e-05,-2.00433085e-06,-3.83993335e-05,-1.6744139e-06,-4.40464664e-06,-2.13951668e-06,-1.75666583e-06,2.03048531e-06,-3.94480709e-09,5.74098371e-09,2.10906625e-07,3.02650227e-07,1.45205285e-08,},
	{ 1.63562232e-05,1.20740931e-05,-1.86105581e-06,3.43490326e-05,6.05825847e-06,7.06161071e-07,1.98865105e-06,-3.01746022e-06,-1.87401726e-06,-1.21826714e-08,2.03045568e-06,-1.98008177e-08,4.88355222e-07,1.48901688e-06,2.8100385e-08,},
	{ -3.15407213e-06,1.04327899e-05,2.04729712e-06,5.62698466e-06,9.91302525e-06,3.98401049e-07,-3.26490919e-07,2.05876931e-06,-5.29111155e-06,1.08678888e-09,1.77250092e-09,2.00654569e-06,4.47161344e-08,4.14518295e-07,-1.3544452e-07,},
	{ -0.000348907202,0.000314489659,-9.79814895e-05,0.000332751126,0.000276947397,-0.000311267592,-3.53020863e-05,7.05450129e-05,3.66262479e-05,4.0082858e-07,8.77334216e-08,1.20709451e-07,0.00102657389,1.38671205e-05,3.1828761e-05,},
	{ -0.000190283001,-0.0001923523,-0.000359131551,-0.000107453348,0.000258576554,9.14961621e-05,-8.1280255e-05,-4.83049105e-05,2.8009286e-06,9.08905402e-07,1.1253333e-06,4.71832044e-07,1.98746194e-05,0.00102957915,1.10534068e-05,},
	{ -4.36858406e-06,3.12491066e-06,-6.78076531e-05,2.44743365e-05,2.21055499e-05,-0.000144033821,-2.16457196e-06,-8.37133476e-08,-6.74226005e-07,1.92376352e-08,2.55265045e-08,-5.72528925e-08,3.2366582e-05,1.07361848e-05,0.000111095067,},
};
