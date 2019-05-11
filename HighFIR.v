// megafunction wizard: %FIR II v18.0%
// GENERATION: XML
// HighFIR.v

// Generated using ACDS version 18.0 614

`timescale 1 ps / 1 ps
module HighFIR (
		input  wire        clk,              //                     clk.clk
		input  wire        reset_n,          //                     rst.reset_n
		input  wire [31:0] ast_sink_data,    //   avalon_streaming_sink.data
		input  wire        ast_sink_valid,   //                        .valid
		input  wire [1:0]  ast_sink_error,   //                        .error
		output wire [31:0] ast_source_data,  // avalon_streaming_source.data
		output wire        ast_source_valid, //                        .valid
		output wire [1:0]  ast_source_error  //                        .error
	);

	HighFIR_0002 highfir_inst (
		.clk              (clk),              //                     clk.clk
		.reset_n          (reset_n),          //                     rst.reset_n
		.ast_sink_data    (ast_sink_data),    //   avalon_streaming_sink.data
		.ast_sink_valid   (ast_sink_valid),   //                        .valid
		.ast_sink_error   (ast_sink_error),   //                        .error
		.ast_source_data  (ast_source_data),  // avalon_streaming_source.data
		.ast_source_valid (ast_source_valid), //                        .valid
		.ast_source_error (ast_source_error)  //                        .error
	);

endmodule
// Retrieval info: <?xml version="1.0"?>
//<!--
//	Generated by Altera MegaWizard Launcher Utility version 1.0
//	************************************************************
//	THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//	************************************************************
//	Copyright (C) 1991-2018 Altera Corporation
//	Any megafunction design, and related net list (encrypted or decrypted),
//	support information, device programming or simulation file, and any other
//	associated documentation or information provided by Altera or a partner
//	under Altera's Megafunction Partnership Program may be used only to
//	program PLD devices (but not masked PLD devices) from Altera.  Any other
//	use of such megafunction design, net list, support information, device
//	programming or simulation file, or any other related documentation or
//	information is prohibited for any other purpose, including, but not
//	limited to modification, reverse engineering, de-compiling, or use with
//	any other silicon devices, unless such use is explicitly licensed under
//	a separate agreement with Altera or a megafunction partner.  Title to
//	the intellectual property, including patents, copyrights, trademarks,
//	trade secrets, or maskworks, embodied in any such megafunction design,
//	net list, support information, device programming or simulation file, or
//	any other related documentation or information provided by Altera or a
//	megafunction partner, remains with Altera, the megafunction partner, or
//	their respective licensors.  No other licenses, including any licenses
//	needed under any third party's intellectual property, are provided herein.
//-->
// Retrieval info: <instance entity-name="altera_fir_compiler_ii" version="18.0" >
// Retrieval info: 	<generic name="filterType" value="single" />
// Retrieval info: 	<generic name="interpFactor" value="1" />
// Retrieval info: 	<generic name="decimFactor" value="1" />
// Retrieval info: 	<generic name="symmetryMode" value="nsym" />
// Retrieval info: 	<generic name="L_bandsFilter" value="1" />
// Retrieval info: 	<generic name="inputChannelNum" value="1" />
// Retrieval info: 	<generic name="clockRate" value="50" />
// Retrieval info: 	<generic name="clockSlack" value="0" />
// Retrieval info: 	<generic name="inputRate" value="0.048" />
// Retrieval info: 	<generic name="coeffReload" value="false" />
// Retrieval info: 	<generic name="baseAddress" value="0" />
// Retrieval info: 	<generic name="readWriteMode" value="read_write" />
// Retrieval info: 	<generic name="backPressure" value="false" />
// Retrieval info: 	<generic name="deviceFamily" value="Cyclone V" />
// Retrieval info: 	<generic name="speedGrade" value="medium" />
// Retrieval info: 	<generic name="delayRAMBlockThreshold" value="20" />
// Retrieval info: 	<generic name="dualMemDistRAMThreshold" value="1280" />
// Retrieval info: 	<generic name="mRAMThreshold" value="1000000" />
// Retrieval info: 	<generic name="hardMultiplierThreshold" value="-1" />
// Retrieval info: 	<generic name="reconfigurable" value="false" />
// Retrieval info: 	<generic name="num_modes" value="2" />
// Retrieval info: 	<generic name="reconfigurable_list" value="0" />
// Retrieval info: 	<generic name="MODE_STRING" value="None Set" />
// Retrieval info: 	<generic name="channelModes" value="0,1,2,3" />
// Retrieval info: 	<generic name="inputType" value="int" />
// Retrieval info: 	<generic name="inputBitWidth" value="32" />
// Retrieval info: 	<generic name="inputFracBitWidth" value="0" />
// Retrieval info: 	<generic name="coeffSetRealValue" value="-0.0680807847141077,0.07382905253305933,0.03080058465958615,0.0038053464294671136,-0.01064490606627942,-0.015895424878840266,-0.015004907520020767,-0.010621371493762577,-0.004876398727394932,6.482094474677158E-4,0.004916222812989621,0.007413855061832384,0.008054364506042322,0.007076393493811378,0.004947500363595254,0.0022216299486693945,-5.432413936317797E-4,-0.002872712106655575,-0.0044399701554521265,-0.005072704868145842,-0.0047894310058274785,-0.003728751285991605,-0.00214543819668108,-3.395356824502681E-4,0.001378305115893813,0.0027459251569793264,0.00357572610828617,0.0037821625968373286,0.0033783182659449934,0.0024710769847779293,0.0012323022940363107,-1.2930663206587713E-4,-0.0013946446870073336,-0.0023789096280175067,-0.002949057276924842,-0.0030427683832115896,-0.0026676613381063797,-0.0018999895731947548,-8.749817127818962E-4,2.4194004667632695E-4,0.001281920440405732,0.0020945328776151443,0.0025658068307937605,0.0026362902108898485,0.0023077242832057676,0.0016458023215314443,7.553452761069495E-4,-2.2727118778958924E-4,-0.0011545683620652455,-0.0018884589800683101,-0.00232248753528664,-0.002404482822176285,-0.002128667168734213,-0.0015413057769481607,-7.320028086500825E-4,1.7152841937006914E-4,0.001038787715277318,0.001738533068193873,0.0021758742389733268,0.0022812888801542297,0.002049069937344195,0.0015169416484697646,7.625931063040598E-4,-9.658915494173896E-5,-9.364986242515656E-4,-0.001631428846895348,-0.0020828031708055358,-0.0022252219952561096,-0.002033737734090449,-0.001544385118129704,-8.311283149106342E-4,4.067766177474602E-6,8.506316200707044E-4,0.0015331271193301705,0.0020827347972051722,0.002145091699272647,0.0021516043966148964,0.0015672971705996798,9.01824167569183E-4,1.422853952207146E-4,-7.856665576588588E-4,-0.0015424161780913453,-0.001992237643364525,-0.002217509039161853,-0.0021630894180402964,-0.001731534141442767,-9.979965999536748E-4,-1.5534113174292956E-4,6.691005525932765E-4,0.0014344800494361085,0.0020341803355204977,0.002325809221498242,0.0022400612446926665,0.001801350834760322,0.0011274782056561673,2.991386152226827E-4,-5.842299306877465E-4,-0.0014217719613392149,-0.0020695475790512663,-0.002402091338084868,-0.0023561796513908583,-0.0019557703669140776,-0.0012745083059398448,-4.121372165551381E-4,5.221474964624578E-4,0.0014046636066687183,0.002100779843742371,0.002495189150615744,0.002510048616943978,0.0021381871925351416,0.001439634093911356,5.244980310346551E-4,-4.685979353657309E-4,-0.0014006805601422191,-0.0021426061761201736,-0.0025890398015547497,-0.002664537162933692,-0.002340401824206666,-0.0016491540345246144,-6.935005029799808E-4,3.8191554964461734E-4,0.00141154826886714,0.0022422232348624444,0.0027492815757487596,0.002853176151801319,0.002538227259964991,0.001844490565838638,8.66018973166275E-4,-2.6678422725858565E-4,-0.0013892857196891335,-0.002329960551548804,-0.002934545944240625,-0.0031105706721748375,-0.0028181091216067104,-0.002093399436253127,-0.0010362028267102655,1.9390177668080532E-4,0.001410024872782471,0.0024403289604217554,0.003126817421407805,0.0033600684778004905,0.003097457174636423,0.0023681722577748076,0.0012697208589156825,-5.025018767566108E-5,-0.0013958816126619306,-0.002567309715046755,-0.0033747674707868954,-0.0037143534170577383,-0.003456540402930611,-0.0027225246068986954,-0.001499077876547291,-7.746047642221912E-5,0.0014178743316623078,0.002741097638851719,0.003647350902970554,0.004061694185383115,0.003880555675776005,0.003085767625868824,0.0018263885352878591,2.6562155175975633E-4,-0.0013997228657551715,-0.0029018834875379914,-0.004002778746764225,-0.004531874309914647,-0.004411533577136114,-0.003620003812040099,-0.002225362426832435,-4.681043903454085E-4,0.0014155256019060896,0.003138863467197921,0.0044597961816499575,0.0051414872530563495,0.005058435753726068,0.0042084641952554805,0.0027044508544593064,7.39098643283957E-4,-0.0014150192628240937,-0.003437829682029489,-0.005010654885487616,-0.005878928890711681,-0.005897500122661388,-0.005030088676330625,-0.0033649525654590655,-0.0011098033490291117,0.0014174932038694354,0.003833379196662588,0.005766928336205379,0.006909090973125501,0.007058431036698515,0.0061412735899692514,0.004238773323213921,0.0015886013443484157,-0.0014363360821837126,-0.0043925473973813614,-0.00682774748976571,-0.008353748472630425,-0.008691941031765361,-0.0077187689115110905,-0.005503557749667782,-0.0023071514658136484,0.0014407094375503506,0.005208070695914134,0.008424941269850626,0.010567397537686991,0.011233485244612932,0.010221364695286025,0.00755672197499461,0.003513468166009412,-0.0014192743692290543,-0.006570605279819325,-0.011168272926961294,-0.01446071999470405,-0.01582266327300814,-0.014857350437858744,-0.011450114483035512,-0.005827173722350783,0.0014230335331983585,0.00941330282048606,0.017046246623445806,0.023128295910808498,0.026486373397739595,0.026153611692129975,0.021461113528539744,0.012205982228279278,-0.0014413402524041095,-0.018717561886933427,-0.03851920966077133,-0.05935588501272371,-0.07956776964108862,-0.09747660522203444,-0.11151195101748591,-0.1204707346837173,0.8764410988061146,-0.1204707346837173,-0.11151195101748591,-0.09747660522203444,-0.07956776964108862,-0.05935588501272371,-0.03851920966077133,-0.018717561886933427,-0.0014413402524041095,0.012205982228279278,0.021461113528539744,0.026153611692129975,0.026486373397739595,0.023128295910808498,0.017046246623445806,0.00941330282048606,0.0014230335331983585,-0.005827173722350783,-0.011450114483035512,-0.014857350437858744,-0.01582266327300814,-0.01446071999470405,-0.011168272926961294,-0.006570605279819325,-0.0014192743692290543,0.003513468166009412,0.00755672197499461,0.010221364695286025,0.011233485244612932,0.010567397537686991,0.008424941269850626,0.005208070695914134,0.0014407094375503506,-0.0023071514658136484,-0.005503557749667782,-0.0077187689115110905,-0.008691941031765361,-0.008353748472630425,-0.00682774748976571,-0.0043925473973813614,-0.0014363360821837126,0.0015886013443484157,0.004238773323213921,0.0061412735899692514,0.007058431036698515,0.006909090973125501,0.005766928336205379,0.003833379196662588,0.0014174932038694354,-0.0011098033490291117,-0.0033649525654590655,-0.005030088676330625,-0.005897500122661388,-0.005878928890711681,-0.005010654885487616,-0.003437829682029489,-0.0014150192628240937,7.39098643283957E-4,0.0027044508544593064,0.0042084641952554805,0.005058435753726068,0.0051414872530563495,0.0044597961816499575,0.003138863467197921,0.0014155256019060896,-4.681043903454085E-4,-0.002225362426832435,-0.003620003812040099,-0.004411533577136114,-0.004531874309914647,-0.004002778746764225,-0.0029018834875379914,-0.0013997228657551715,2.6562155175975633E-4,0.0018263885352878591,0.003085767625868824,0.003880555675776005,0.004061694185383115,0.003647350902970554,0.002741097638851719,0.0014178743316623078,-7.746047642221912E-5,-0.001499077876547291,-0.0027225246068986954,-0.003456540402930611,-0.0037143534170577383,-0.0033747674707868954,-0.002567309715046755,-0.0013958816126619306,-5.025018767566108E-5,0.0012697208589156825,0.0023681722577748076,0.003097457174636423,0.0033600684778004905,0.003126817421407805,0.0024403289604217554,0.001410024872782471,1.9390177668080532E-4,-0.0010362028267102655,-0.002093399436253127,-0.0028181091216067104,-0.0031105706721748375,-0.002934545944240625,-0.002329960551548804,-0.0013892857196891335,-2.6678422725858565E-4,8.66018973166275E-4,0.001844490565838638,0.002538227259964991,0.002853176151801319,0.0027492815757487596,0.0022422232348624444,0.00141154826886714,3.8191554964461734E-4,-6.935005029799808E-4,-0.0016491540345246144,-0.002340401824206666,-0.002664537162933692,-0.0025890398015547497,-0.0021426061761201736,-0.0014006805601422191,-4.685979353657309E-4,5.244980310346551E-4,0.001439634093911356,0.0021381871925351416,0.002510048616943978,0.002495189150615744,0.002100779843742371,0.0014046636066687183,5.221474964624578E-4,-4.121372165551381E-4,-0.0012745083059398448,-0.0019557703669140776,-0.0023561796513908583,-0.002402091338084868,-0.0020695475790512663,-0.0014217719613392149,-5.842299306877465E-4,2.991386152226827E-4,0.0011274782056561673,0.001801350834760322,0.0022400612446926665,0.002325809221498242,0.0020341803355204977,0.0014344800494361085,6.691005525932765E-4,-1.5534113174292956E-4,-9.979965999536748E-4,-0.001731534141442767,-0.0021630894180402964,-0.002217509039161853,-0.001992237643364525,-0.0015424161780913453,-7.856665576588588E-4,1.422853952207146E-4,9.01824167569183E-4,0.0015672971705996798,0.0021516043966148964,0.002145091699272647,0.0020827347972051722,0.0015331271193301705,8.506316200707044E-4,4.067766177474602E-6,-8.311283149106342E-4,-0.001544385118129704,-0.002033737734090449,-0.0022252219952561096,-0.0020828031708055358,-0.001631428846895348,-9.364986242515656E-4,-9.658915494173896E-5,7.625931063040598E-4,0.0015169416484697646,0.002049069937344195,0.0022812888801542297,0.0021758742389733268,0.001738533068193873,0.001038787715277318,1.7152841937006914E-4,-7.320028086500825E-4,-0.0015413057769481607,-0.002128667168734213,-0.002404482822176285,-0.00232248753528664,-0.0018884589800683101,-0.0011545683620652455,-2.2727118778958924E-4,7.553452761069495E-4,0.0016458023215314443,0.0023077242832057676,0.0026362902108898485,0.0025658068307937605,0.0020945328776151443,0.001281920440405732,2.4194004667632695E-4,-8.749817127818962E-4,-0.0018999895731947548,-0.0026676613381063797,-0.0030427683832115896,-0.002949057276924842,-0.0023789096280175067,-0.0013946446870073336,-1.2930663206587713E-4,0.0012323022940363107,0.0024710769847779293,0.0033783182659449934,0.0037821625968373286,0.00357572610828617,0.0027459251569793264,0.001378305115893813,-3.395356824502681E-4,-0.00214543819668108,-0.003728751285991605,-0.0047894310058274785,-0.005072704868145842,-0.0044399701554521265,-0.002872712106655575,-5.432413936317797E-4,0.0022216299486693945,0.004947500363595254,0.007076393493811378,0.008054364506042322,0.007413855061832384,0.004916222812989621,6.482094474677158E-4,-0.004876398727394932,-0.010621371493762577,-0.015004907520020767,-0.015895424878840266,-0.01064490606627942,0.0038053464294671136,0.03080058465958615,0.07382905253305933,-0.0680807847141077" />
// Retrieval info: 	<generic name="coeffSetRealValueImag" value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0530093, -0.04498, 0.0, 0.0749693, 0.159034, 0.224907, 0.249809, 0.224907, 0.159034, 0.0749693, 0.0, -0.04498, -0.0530093, -0.0321283, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0" />
// Retrieval info: 	<generic name="coeffScaling" value="auto" />
// Retrieval info: 	<generic name="coeffType" value="int" />
// Retrieval info: 	<generic name="coeffBitWidth" value="10" />
// Retrieval info: 	<generic name="coeffFracBitWidth" value="0" />
// Retrieval info: 	<generic name="coeffComplex" value="false" />
// Retrieval info: 	<generic name="karatsuba" value="false" />
// Retrieval info: 	<generic name="outType" value="int" />
// Retrieval info: 	<generic name="outMSBRound" value="sat" />
// Retrieval info: 	<generic name="outMsbBitRem" value="19" />
// Retrieval info: 	<generic name="outLSBRound" value="trunc" />
// Retrieval info: 	<generic name="outLsbBitRem" value="0" />
// Retrieval info: 	<generic name="bankCount" value="1" />
// Retrieval info: 	<generic name="bankDisplay" value="0" />
// Retrieval info: </instance>
// IPFS_FILES : HighFIR.vo
// RELATED_FILES: HighFIR.v, dspba_library_package.vhd, dspba_library.vhd, auk_dspip_math_pkg_hpfir.vhd, auk_dspip_lib_pkg_hpfir.vhd, auk_dspip_avalon_streaming_controller_hpfir.vhd, auk_dspip_avalon_streaming_sink_hpfir.vhd, auk_dspip_avalon_streaming_source_hpfir.vhd, auk_dspip_roundsat_hpfir.vhd, altera_avalon_sc_fifo.v, HighFIR_0002_rtl_core.vhd, HighFIR_0002_ast.vhd, HighFIR_0002.vhd