#ifndef INCLUDE_RES_NAOSVG_H
#define INCLUDE_RES_NAOSVG_H


#include <string>
#include <sstream>


std::string generateNaoSVG(int r, int g, int b) {
	std::stringstream s;
	s << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>" << "\n";
	s << "<svg" << "\n";
	s << "   xmlns:dc=\"http://purl.org/dc/elements/1.1/\"" << "\n";
	s << "   xmlns:cc=\"http://creativecommons.org/ns#\"" << "\n";
	s << "   xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\"" << "\n";
	s << "   xmlns:svg=\"http://www.w3.org/2000/svg\"" << "\n";
	s << "   xmlns=\"http://www.w3.org/2000/svg\"" << "\n";
	s << "   xmlns:xlink=\"http://www.w3.org/1999/xlink\"" << "\n";
	s << "   viewBox=\"0 0 128 101.79109\"" << "\n";
	s << "   height=\"101.79108\"" << "\n";
	s << "   width=\"128\"" << "\n";
	s << "   id=\"svg4572\"" << "\n";
	s << "   version=\"1.1\">" << "\n";
	s << "  <metadata" << "\n";
	s << "     id=\"metadata4578\">" << "\n";
	s << "    <rdf:RDF>" << "\n";
	s << "      <cc:Work" << "\n";
	s << "         rdf:about=\"\">" << "\n";
	s << "        <dc:format>image/svg+xml</dc:format>" << "\n";
	s << "        <dc:type" << "\n";
	s << "           rdf:resource=\"http://purl.org/dc/dcmitype/StillImage\" />" << "\n";
	s << "        <dc:title></dc:title>" << "\n";
	s << "      </cc:Work>" << "\n";
	s << "    </rdf:RDF>" << "\n";
	s << "  </metadata>" << "\n";
	s << "  <defs" << "\n";
	s << "     id=\"defs4576\">" << "\n";
	s << "    <linearGradient" << "\n";
	s << "       id=\"linearGradient980-6\">" << "\n";
	s << "      <stop" << "\n";
	s << "         id=\"stop976\"" << "\n";
	s << "         offset=\"0\"" << "\n";
	s << "         style=\"stop-color:#ffffff;stop-opacity:1;\" />" << "\n";
	s << "      <stop" << "\n";
	s << "         id=\"stop978\"" << "\n";
	s << "         offset=\"1\"" << "\n";
	s << "         style=\"stop-color:#a5a5a5;stop-opacity:1\" />" << "\n";
	s << "    </linearGradient>" << "\n";
	s << "    <linearGradient" << "\n";
	s << "       id=\"linearGradient869\">" << "\n";
	s << "      <stop" << "\n";
	s << "         style=\"stop-color:#ffffff;stop-opacity:0.33613446\"" << "\n";
	s << "         offset=\"0\"" << "\n";
	s << "         id=\"stop865\" />" << "\n";
	s << "      <stop" << "\n";
	s << "         style=\"stop-color:#ffffff;stop-opacity:0\"" << "\n";
	s << "         offset=\"1\"" << "\n";
	s << "         id=\"stop867\" />" << "\n";
	s << "    </linearGradient>" << "\n";
	s << "    <linearGradient" << "\n";
	s << "       id=\"linearGradient855\">" << "\n";
	s << "      <stop" << "\n";
	s << "         id=\"stop851\"" << "\n";
	s << "         offset=\"0\"" << "\n";
	s << "         style=\"stop-color:#ffffff;stop-opacity:0.4579832\" />" << "\n";
	s << "      <stop" << "\n";
	s << "         id=\"stop853\"" << "\n";
	s << "         offset=\"1\"" << "\n";
	s << "         style=\"stop-color:#ffffff;stop-opacity:0\" />" << "\n";
	s << "    </linearGradient>" << "\n";
	s << "    <linearGradient" << "\n";
	s << "       id=\"linearGradient4704\">" << "\n";
	s << "      <stop" << "\n";
	s << "         id=\"stop4700\"" << "\n";
	s << "         offset=\"0\"" << "\n";
	s << "         style=\"stop-color:#a7a7a7;stop-opacity:1\" />" << "\n";
	s << "      <stop" << "\n";
	s << "         id=\"stop4702\"" << "\n";
	s << "         offset=\"1\"" << "\n";
	s << "         style=\"stop-color:#ffffff;stop-opacity:1\" />" << "\n";
	s << "    </linearGradient>" << "\n";
	s << "    <linearGradient" << "\n";
	s << "       id=\"linearGradient4679-3\">" << "\n";
	s << "      <stop" << "\n";
	s << "         id=\"stop4675\"" << "\n";
	s << "         offset=\"0\"" << "\n";
	s << "         style=\"stop-color:#ffffff;stop-opacity:0.29411766\" />" << "\n";
	s << "      <stop" << "\n";
	s << "         id=\"stop4677\"" << "\n";
	s << "         offset=\"1\"" << "\n";
	s << "         style=\"stop-color:#fafbff;stop-opacity:0\" />" << "\n";
	s << "    </linearGradient>" << "\n";
	s << "    <radialGradient" << "\n";
	s << "       gradientUnits=\"userSpaceOnUse\"" << "\n";
	s << "       gradientTransform=\"matrix(1.0378583,0.00663606,-0.01058463,1.3294848,-2.0331671,-16.077036)\"" << "\n";
	s << "       r=\"34.667969\"" << "\n";
	s << "       fy=\"36.829632\"" << "\n";
	s << "       fx=\"64.355507\"" << "\n";
	s << "       cy=\"36.829632\"" << "\n";
	s << "       cx=\"64.355507\"" << "\n";
	s << "       id=\"radialGradient4681\"" << "\n";
	s << "       xlink:href=\"#linearGradient4679-3\" />" << "\n";
	s << "    <linearGradient" << "\n";
	s << "       gradientUnits=\"userSpaceOnUse\"" << "\n";
	s << "       y2=\"64.733047\"" << "\n";
	s << "       x2=\"46.845825\"" << "\n";
	s << "       y1=\"45.817944\"" << "\n";
	s << "       x1=\"46.845825\"" << "\n";
	s << "       id=\"linearGradient4706\"" << "\n";
	s << "       xlink:href=\"#linearGradient4704\" />" << "\n";
	s << "    <linearGradient" << "\n";
	s << "       gradientTransform=\"translate(36.239222)\"" << "\n";
	s << "       y2=\"64.733047\"" << "\n";
	s << "       x2=\"46.845825\"" << "\n";
	s << "       y1=\"45.817944\"" << "\n";
	s << "       x1=\"46.845825\"" << "\n";
	s << "       gradientUnits=\"userSpaceOnUse\"" << "\n";
	s << "       id=\"linearGradient4730\"" << "\n";
	s << "       xlink:href=\"#linearGradient4704\" />" << "\n";
	s << "    <radialGradient" << "\n";
	s << "       gradientUnits=\"userSpaceOnUse\"" << "\n";
	s << "       gradientTransform=\"matrix(0.45971561,0,0,0.49675434,10.626372,44.290238)\"" << "\n";
	s << "       r=\"37.299881\"" << "\n";
	s << "       fy=\"42.040554\"" << "\n";
	s << "       fx=\"21.743534\"" << "\n";
	s << "       cy=\"42.040554\"" << "\n";
	s << "       cx=\"21.743534\"" << "\n";
	s << "       id=\"radialGradient857\"" << "\n";
	s << "       xlink:href=\"#linearGradient855\" />" << "\n";
	s << "    <radialGradient" << "\n";
	s << "       r=\"37.299881\"" << "\n";
	s << "       fy=\"42.040554\"" << "\n";
	s << "       fx=\"21.743534\"" << "\n";
	s << "       cy=\"42.040554\"" << "\n";
	s << "       cx=\"21.743534\"" << "\n";
	s << "       gradientTransform=\"matrix(0.43127959,0,0,0.46602727,101.315,45.607148)\"" << "\n";
	s << "       gradientUnits=\"userSpaceOnUse\"" << "\n";
	s << "       id=\"radialGradient861\"" << "\n";
	s << "       xlink:href=\"#linearGradient869\" />" << "\n";
	s << "    <linearGradient" << "\n";
	s << "       gradientUnits=\"userSpaceOnUse\"" << "\n";
	s << "       y2=\"102.39033\"" << "\n";
	s << "       x2=\"65.973267\"" << "\n";
	s << "       y1=\"87.906158\"" << "\n";
	s << "       x1=\"54.532745\"" << "\n";
	s << "       id=\"linearGradient982\"" << "\n";
	s << "       xlink:href=\"#linearGradient980-6\" />" << "\n";
	s << "    <linearGradient" << "\n";
	s << "       y2=\"102.39033\"" << "\n";
	s << "       x2=\"65.973267\"" << "\n";
	s << "       y1=\"87.906158\"" << "\n";
	s << "       x1=\"54.532745\"" << "\n";
	s << "       gradientUnits=\"userSpaceOnUse\"" << "\n";
	s << "       id=\"linearGradient989\"" << "\n";
	s << "       xlink:href=\"#linearGradient980-6\" />" << "\n";
	s << "  </defs>" << "\n";
	s << "  <g" << "\n";
	s << "     style=\"fill:url(#linearGradient982);fill-opacity:1;stroke:#6e6e6e;stroke-opacity:1\"" << "\n";
	s << "     transform=\"matrix(5.9687359,0,0,6.0324467,-291.08747,-526.08957)\"" << "\n";
	s << "     id=\"g864\">" << "\n";
	s << "    <path" << "\n";
	s << "       id=\"path862\"" << "\n";
	s << "       d=\"m 51.304688,90.984375 c -1.170723,0.226091 -1.688809,1.368984 -1.794922,2.480469 -0.172751,1.809496 -0.204503,3.65156 0.107422,5.439453 0.195485,1.120483 1.175564,1.714573 2.15625,2.125003 2.257332,0.94472 5.171382,2.71326 7.714953,2.73047 2.54357,0.0172 5.374945,-1.71249 7.644847,-2.62659 0.986149,-0.39713 1.974173,-0.97791 2.184795,-2.095645 0.336079,-1.78351 0.329246,-3.625835 0.180983,-5.437502 -0.09107,-1.112819 -0.593649,-2.262615 -1.761208,-2.50452 -1.463489,-0.303219 -2.59601,-1.3167 -3.879555,-1.999922 -1.122,-0.597233 -2.981181,-1.119716 -4.369862,-1.129111 -1.388681,-0.0094 -3.121845,0.541597 -4.331972,1.164912 -1.210128,0.623315 -2.384274,1.569586 -3.851731,1.852983 z\"" << "\n";
	s << "       style=\"fill:url(#linearGradient989);fill-opacity:1;stroke:#6e6e6e;stroke-width:0.15876687;stroke-opacity:1\" />" << "\n";
	s << "  </g>" << "\n";
	s << "  <path" << "\n";
	s << "     style=\"fill:#000000;fill-opacity:0.50420173;fill-rule:evenodd;stroke:none;stroke-width:0.982526px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1\"" << "\n";
	s << "     d=\"m 63.999924,4.2682404 c -9.125895,-0.055361 -20.516348,3.1887531 -28.468853,6.8596356 -2.50283,1.155308 -4.987027,2.500175 -7.510474,3.851279 0.272116,17.009747 11.220236,26.880835 20.561063,26.880835 h 30.836528 c 9.3703,0 20.359892,-9.932093 20.565117,-27.0407 -2.390021,-1.364184 -4.786068,-2.718574 -7.26722,-3.902144 C 85.342718,7.3998674 73.125819,4.3235704 63.999924,4.2682404 Z\"" << "\n";
	s << "     id=\"path4746\" />" << "\n";
	s << "  <path" << "\n";
	s << "     id=\"path4638\"" << "\n";
	s << "     d=\"m 63.999924,3.3381352 c -9.125895,-0.055361 -20.516348,3.188753 -28.468853,6.8596358 -2.50283,1.155307 -4.987027,2.500176 -7.510474,3.851278 0.272116,17.009748 11.220236,26.880836 20.561063,26.880836 h 30.836528 c 9.3703,0 20.359892,-9.932093 20.565117,-27.040699 C 97.593284,12.525001 95.197237,11.170611 92.716085,9.9870401 85.342718,6.4697622 73.125819,3.3934652 63.999924,3.3381352 Z\"" << "\n";
	s << "     style=\"fill:rgb(" << r << ", " << g << ", " << b << ");fill-opacity:1;fill-rule:evenodd;stroke:none;stroke-width:0.982526px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1\" />" << "\n";
	s << "  <path" << "\n";
	s << "     style=\"fill:url(#radialGradient4681);fill-opacity:1;fill-rule:evenodd;stroke:none;stroke-width:0.982526px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1\"" << "\n";
	s << "     d=\"m 63.999924,3.3381352 c -9.125895,-0.055361 -20.516348,3.188753 -28.468853,6.8596358 -2.50283,1.155307 -4.987027,2.500176 -7.510474,3.851278 0.272116,17.009748 11.220236,26.880836 20.561063,26.880836 h 30.836528 c 9.3703,0 20.359892,-9.932093 20.565117,-27.040699 C 97.593284,12.525001 95.197237,11.170611 92.716085,9.9870401 85.342718,6.4697622 73.125819,3.3934652 63.999924,3.3381352 Z\"" << "\n";
	s << "     id=\"path4673\" />" << "\n";
	s << "  <g" << "\n";
	s << "     id=\"g4646\"" << "\n";
	s << "     transform=\"matrix(6.3317517,0,0,6.3317517,-312.66572,-555.19082)\" />" << "\n";
	s << "  <g" << "\n";
	s << "     style=\"stroke:none;stroke-opacity:1;stroke-width:0.23334524;stroke-miterlimit:4;stroke-dasharray:none\"" << "\n";
	s << "     transform=\"matrix(1.2856487,0,0,1.2856487,-17.772731,-14.376559)\"" << "\n";
	s << "     id=\"g4758\">" << "\n";
	s << "    <circle" << "\n";
	s << "       style=\"opacity:1;fill:url(#linearGradient4706);fill-opacity:1;stroke:none;stroke-width:0.23334524;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\"" << "\n";
	s << "       id=\"path4690\"" << "\n";
	s << "       cx=\"46.492271\"" << "\n";
	s << "       cy=\"55.629051\"" << "\n";
	s << "       r=\"9.7227182\" />" << "\n";
	s << "    <circle" << "\n";
	s << "       style=\"opacity:1;fill:#000000;fill-opacity:1;stroke:none;stroke-width:0.23334524;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\"" << "\n";
	s << "       id=\"path4708\"" << "\n";
	s << "       cx=\"46.492275\"" << "\n";
	s << "       cy=\"55.805828\"" << "\n";
	s << "       r=\"4.6845822\" />" << "\n";
	s << "    <circle" << "\n";
	s << "       style=\"opacity:1;fill:#ffffff;fill-opacity:1;stroke:none;stroke-width:0.23334524;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\"" << "\n";
	s << "       id=\"path4742\"" << "\n";
	s << "       cx=\"44.98967\"" << "\n";
	s << "       cy=\"54.038059\"" << "\n";
	s << "       r=\"1.4142135\" />" << "\n";
	s << "  </g>" << "\n";
	s << "  <g" << "\n";
	s << "     style=\"stroke:none;stroke-opacity:1;stroke-width:0.23334524;stroke-miterlimit:4;stroke-dasharray:none\"" << "\n";
	s << "     transform=\"matrix(1.2856487,0,0,1.2856487,-20.363638,-14.376559)\"" << "\n";
	s << "     id=\"g4763\">" << "\n";
	s << "    <circle" << "\n";
	s << "       r=\"9.7227182\"" << "\n";
	s << "       cy=\"55.629051\"" << "\n";
	s << "       cx=\"82.731491\"" << "\n";
	s << "       id=\"circle4722\"" << "\n";
	s << "       style=\"opacity:1;fill:url(#linearGradient4730);fill-opacity:1;stroke:none;stroke-width:0.23334524;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\" />" << "\n";
	s << "    <circle" << "\n";
	s << "       r=\"4.6845822\"" << "\n";
	s << "       cy=\"55.805828\"" << "\n";
	s << "       cx=\"82.731499\"" << "\n";
	s << "       id=\"circle4724\"" << "\n";
	s << "       style=\"opacity:1;fill:#000000;fill-opacity:1;stroke:none;stroke-width:0.23334524;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\" />" << "\n";
	s << "    <circle" << "\n";
	s << "       r=\"1.4142135\"" << "\n";
	s << "       cy=\"53.861282\"" << "\n";
	s << "       cx=\"81.40567\"" << "\n";
	s << "       id=\"circle4744\"" << "\n";
	s << "       style=\"opacity:1;fill:#ffffff;fill-opacity:1;stroke:none;stroke-width:0.23334524;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\" />" << "\n";
	s << "  </g>" << "\n";
	s << "  <path" << "\n";
	s << "     style=\"opacity:1;fill:#ffffff;fill-opacity:1;stroke:none;stroke-width:25.02394295;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\"" << "\n";
	s << "     d=\"m 60.999998,79.093329 h 6 l -1.263158,5.205592 h -3.473684 z\"" << "\n";
	s << "     id=\"path4765\" />" << "\n";
	s << "  <path" << "\n";
	s << "     id=\"rect4752\"" << "\n";
	s << "     d=\"m 60.999998,79.093329 h 6 l -1.263158,4.736842 h -3.473684 z\"" << "\n";
	s << "     style=\"opacity:1;fill:#000000;fill-opacity:1;stroke:none;stroke-width:23.87069893;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\" />" << "\n";
	s << "  <circle" << "\n";
	s << "     r=\"2.2316294\"" << "\n";
	s << "     cy=\"39.434448\"" << "\n";
	s << "     cx=\"64.000114\"" << "\n";
	s << "     id=\"path4767\"" << "\n";
	s << "     style=\"opacity:1;fill:#000000;fill-opacity:1;stroke:none;stroke-width:35.51370621;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\" />" << "\n";
	s << "  <ellipse" << "\n";
	s << "     ry=\"18.528877\"" << "\n";
	s << "     rx=\"17.147337\"" << "\n";
	s << "     cy=\"65.174065\"" << "\n";
	s << "     cx=\"20.622213\"" << "\n";
	s << "     id=\"path849\"" << "\n";
	s << "     style=\"opacity:1;fill:url(#radialGradient857);fill-opacity:1;stroke:none;stroke-width:17.3750782;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\" />" << "\n";
	s << "  <ellipse" << "\n";
	s << "     style=\"opacity:1;fill:url(#radialGradient861);fill-opacity:1;stroke:none;stroke-width:16.30033112;stroke-linecap:square;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1\"" << "\n";
	s << "     id=\"ellipse859\"" << "\n";
	s << "     cx=\"110.69254\"" << "\n";
	s << "     cy=\"65.199196\"" << "\n";
	s << "     rx=\"16.086678\"" << "\n";
	s << "     ry=\"17.382761\" />" << "\n";
	s << "  <path" << "\n";
	s << "     id=\"path948\"" << "\n";
	s << "     d=\"M 93.927075,10.587663 C 93.078663,23.86905 84.304408,31.533047 76.77326,31.533047 H 50.946187 c -7.449683,0 -16.112982,-7.500492 -17.120782,-20.517777 -1.929254,0.951794 -3.85539,1.989866 -5.804806,3.033621 0.272116,17.009747 11.220995,26.88049 20.561823,26.88049 h 30.835392 c 9.3703,0 20.360266,-9.931548 20.565491,-27.040154 -1.998112,-1.140488 -4.00349,-2.268858 -6.05623,-3.301564 z\"" << "\n";
	s << "     style=\"fill:#ffffff;fill-opacity:0.10084037;fill-rule:evenodd;stroke:none;stroke-width:0.982526px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1\" />" << "\n";
	s << "</svg>" << "\n";
	return s.str();
}


#endif
