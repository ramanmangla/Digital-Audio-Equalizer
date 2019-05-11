// megafunction wizard: %FIR II v18.0%
// GENERATION: XML
// LowFIR500.v

// Generated using ACDS version 18.0 614

`timescale 1 ps / 1 ps
module LowFIR500 (
		input  wire        clk,              //                     clk.clk
		input  wire        reset_n,          //                     rst.reset_n
		input  wire [31:0] ast_sink_data,    //   avalon_streaming_sink.data
		input  wire        ast_sink_valid,   //                        .valid
		input  wire [1:0]  ast_sink_error,   //                        .error
		output wire [31:0] ast_source_data,  // avalon_streaming_source.data
		output wire        ast_source_valid, //                        .valid
		output wire [1:0]  ast_source_error  //                        .error
	);

	LowFIR500_0002 lowfir500_inst (
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
// Retrieval info: 	<generic name="coeffSetRealValue" value="0.005386907564224238,8.721820584681918E-4,9.396483665027597E-4,0.0010083663299140725,0.001078061381627579,0.0011485042821481096,0.0012195191539385937,0.0012907943377894076,0.00136204083290026,0.0014330790285197704,0.0015035176917432092,0.001573116459746048,0.0016415254938129812,0.0017085294475605677,0.0017738186591553986,0.0018370976998262685,0.0018980510358292558,0.001956359880201869,0.0020117809676790392,0.0020640024499177525,0.0021127691039435808,0.0021577168462034316,0.002198566035895059,0.002235017480737369,0.002266805094889138,0.0022936787019615405,0.002315241458090272,0.0023313495386670284,0.002341591816889338,0.002346188149645139,0.0023443379335936244,0.0023362438265207607,0.0023218538239033575,0.0023008016011791836,0.002273034814167911,0.002238728125864494,0.0021977010257414266,0.0021496133443670348,0.002094915038132874,0.0020332686615493496,0.001964761658147786,0.0018892642021272066,0.0018069266294138426,0.0017178284445394318,0.0016221001349896722,0.001519935477571203,0.0014115762755368799,0.0012974263997087769,0.0011778404449295512,0.00105321579817779,9.238122408598765E-4,7.898789097319762E-4,6.516062671571733E-4,5.091593992025311E-4,3.6277568252744653E-4,2.1272386010278316E-4,5.9647673293663376E-5,-9.563905795277581E-5,-2.519331343938839E-4,-4.083852247508939E-4,-5.646191368352069E-4,-7.21228838137312E-4,-8.796560912120939E-4,-0.0010401173266335441,-0.0011977985137946595,-0.0013415943128907686,-0.001499590403801977,-0.0016452478641791094,-0.0017894314409218647,-0.001929294765820224,-0.002064431835499715,-0.0021942562949303075,-0.0023183097844253417,-0.002436093203610842,-0.002547010843870901,-0.0026506772443507295,-0.0027466076894877354,-0.0028344640465929765,-0.00291379294519166,-0.002984245752702126,-0.003045476333918259,-0.003097202813506391,-0.0031391955954083333,-0.003171185631109725,-0.0031929966970383294,-0.003204405665579639,-0.003205343819825033,-0.003195699438510357,-0.0031754470781840912,-0.0031445615224370287,-0.0031030512654477375,-0.0030511570203944796,-0.002988922893330949,-0.002916807649861355,-0.0028344535286925166,-0.002742978712643436,-0.0026421138735489413,-0.0025322155347751803,-0.0024139925587284964,-0.002287731608188228,-0.002153643595261834,-0.002012447438997039,-0.001864971336310137,-0.0017113953832106018,-0.0015526145320768638,-0.0013892416158122695,-0.0012221462353396828,-0.0010519258658812788,-8.792526851437551E-4,-7.048029740973158E-4,-5.29320058757924E-4,-3.5363754995143223E-4,-1.7853695495924615E-4,-4.9046850752082956E-6,1.6641734235394107E-4,3.345010935336202E-4,4.985346697901011E-4,6.578047437426867E-4,8.116865286074359E-4,9.595675436274102E-4,0.001100558753157621,0.0012337217219512634,0.0013579443710480781,0.001472631034905271,0.0015773627482130296,0.0016721212424355078,0.0017564104698761039,0.001828609014608391,0.0018872798422996705,0.0019335974979558714,0.0019698507174638223,0.0019884286033363587,0.001995957137054489,0.0019885227258090795,0.0019670384698314575,0.0019311796912232612,0.0018809420553588428,0.0018162376544757547,0.0017371067294500609,0.0016437737040905652,0.0015363788643729624,0.0014152298860263433,0.0012805621931235786,0.001132801181109369,9.7236649495611E-4,7.997867216727501E-4,6.156294355405414E-4,4.205078146456257E-4,2.1516859219894953E-4,3.616592185475539E-7,-2.23011114116841E-4,-4.541224381624216E-4,-6.920149586282345E-4,-9.357517642345615E-4,-0.0011842854035838482,-0.001436496093015484,-0.0016914098556692627,-0.0019477592424908,-0.0022047139526965615,-0.0024602234787805665,-0.0027142340782490616,-0.002964536041014642,-0.003210025027284905,-0.0034498588220519977,-0.003682519938436567,-0.003906465701805126,-0.004120716586487467,-0.00432427565486979,-0.004515484973087814,-0.004693212584204137,-0.004856134728901808,-0.0050032525306604675,-0.005133371071111627,-0.005245394392187159,-0.005338248437354215,-0.005410964576528469,-0.005462676248030469,-0.005492438288632956,-0.0054993615254787425,-0.005482530485096796,-0.005441271911412816,-0.005375049879929755,-0.0052834651973328395,-0.005166095070260372,-0.005022390701478069,-0.004851936699211438,-0.004654383662830647,-0.0044298858565415804,-0.004178493194772413,-0.003900435723075637,-0.0035954125713604764,-0.0032633247018200064,-0.002905074718976291,-0.0025217162748480374,-0.002112866475318758,-0.0016780910801327657,-0.0012212044856537746,-7.398370806870694E-4,-2.3679208955354673E-4,2.876639053554386E-4,8.323257456122001E-4,0.0013961701056613108,0.0019779135588423986,0.002576238582137136,0.0031898510509241118,0.00381727063809513,0.004457053429949258,0.005107572151411372,0.005767272533124925,0.006434492675844113,0.007107580718922249,0.007784816471770923,0.008464416913804612,0.009144627764178648,0.009823609561431947,0.01049961403951742,0.011170754420093331,0.01183523161750374,0.012491165609410701,0.013136778394479447,0.01377035604898057,0.014390025708981386,0.014994201480456847,0.01558069864777363,0.01614905372051449,0.01669580324467329,0.017220982444620957,0.017722612567738945,0.018198653661105773,0.018648156858710532,0.019070216729979016,0.019463297474323307,0.019825941774345192,0.020157401532944774,0.020456665287805334,0.020722966534008663,0.020955301585875663,0.021152978781117347,0.02131543326913557,0.02144231941774588,0.021533333278338066,0.021588119832984514,0.021606433902324335,0.021588119832984514,0.021533333278338066,0.02144231941774588,0.02131543326913557,0.021152978781117347,0.020955301585875663,0.020722966534008663,0.020456665287805334,0.020157401532944774,0.019825941774345192,0.019463297474323307,0.019070216729979016,0.018648156858710532,0.018198653661105773,0.017722612567738945,0.017220982444620957,0.01669580324467329,0.01614905372051449,0.01558069864777363,0.014994201480456847,0.014390025708981386,0.01377035604898057,0.013136778394479447,0.012491165609410701,0.01183523161750374,0.011170754420093331,0.01049961403951742,0.009823609561431947,0.009144627764178648,0.008464416913804612,0.007784816471770923,0.007107580718922249,0.006434492675844113,0.005767272533124925,0.005107572151411372,0.004457053429949258,0.00381727063809513,0.0031898510509241118,0.002576238582137136,0.0019779135588423986,0.0013961701056613108,8.323257456122001E-4,2.876639053554386E-4,-2.3679208955354673E-4,-7.398370806870694E-4,-0.0012212044856537746,-0.0016780910801327657,-0.002112866475318758,-0.0025217162748480374,-0.002905074718976291,-0.0032633247018200064,-0.0035954125713604764,-0.003900435723075637,-0.004178493194772413,-0.0044298858565415804,-0.004654383662830647,-0.004851936699211438,-0.005022390701478069,-0.005166095070260372,-0.0052834651973328395,-0.005375049879929755,-0.005441271911412816,-0.005482530485096796,-0.0054993615254787425,-0.005492438288632956,-0.005462676248030469,-0.005410964576528469,-0.005338248437354215,-0.005245394392187159,-0.005133371071111627,-0.0050032525306604675,-0.004856134728901808,-0.004693212584204137,-0.004515484973087814,-0.00432427565486979,-0.004120716586487467,-0.003906465701805126,-0.003682519938436567,-0.0034498588220519977,-0.003210025027284905,-0.002964536041014642,-0.0027142340782490616,-0.0024602234787805665,-0.0022047139526965615,-0.0019477592424908,-0.0016914098556692627,-0.001436496093015484,-0.0011842854035838482,-9.357517642345615E-4,-6.920149586282345E-4,-4.541224381624216E-4,-2.23011114116841E-4,3.616592185475539E-7,2.1516859219894953E-4,4.205078146456257E-4,6.156294355405414E-4,7.997867216727501E-4,9.7236649495611E-4,0.001132801181109369,0.0012805621931235786,0.0014152298860263433,0.0015363788643729624,0.0016437737040905652,0.0017371067294500609,0.0018162376544757547,0.0018809420553588428,0.0019311796912232612,0.0019670384698314575,0.0019885227258090795,0.001995957137054489,0.0019884286033363587,0.0019698507174638223,0.0019335974979558714,0.0018872798422996705,0.001828609014608391,0.0017564104698761039,0.0016721212424355078,0.0015773627482130296,0.001472631034905271,0.0013579443710480781,0.0012337217219512634,0.001100558753157621,9.595675436274102E-4,8.116865286074359E-4,6.578047437426867E-4,4.985346697901011E-4,3.345010935336202E-4,1.6641734235394107E-4,-4.9046850752082956E-6,-1.7853695495924615E-4,-3.5363754995143223E-4,-5.29320058757924E-4,-7.048029740973158E-4,-8.792526851437551E-4,-0.0010519258658812788,-0.0012221462353396828,-0.0013892416158122695,-0.0015526145320768638,-0.0017113953832106018,-0.001864971336310137,-0.002012447438997039,-0.002153643595261834,-0.002287731608188228,-0.0024139925587284964,-0.0025322155347751803,-0.0026421138735489413,-0.002742978712643436,-0.0028344535286925166,-0.002916807649861355,-0.002988922893330949,-0.0030511570203944796,-0.0031030512654477375,-0.0031445615224370287,-0.0031754470781840912,-0.003195699438510357,-0.003205343819825033,-0.003204405665579639,-0.0031929966970383294,-0.003171185631109725,-0.0031391955954083333,-0.003097202813506391,-0.003045476333918259,-0.002984245752702126,-0.00291379294519166,-0.0028344640465929765,-0.0027466076894877354,-0.0026506772443507295,-0.002547010843870901,-0.002436093203610842,-0.0023183097844253417,-0.0021942562949303075,-0.002064431835499715,-0.001929294765820224,-0.0017894314409218647,-0.0016452478641791094,-0.001499590403801977,-0.0013415943128907686,-0.0011977985137946595,-0.0010401173266335441,-8.796560912120939E-4,-7.21228838137312E-4,-5.646191368352069E-4,-4.083852247508939E-4,-2.519331343938839E-4,-9.563905795277581E-5,5.9647673293663376E-5,2.1272386010278316E-4,3.6277568252744653E-4,5.091593992025311E-4,6.516062671571733E-4,7.898789097319762E-4,9.238122408598765E-4,0.00105321579817779,0.0011778404449295512,0.0012974263997087769,0.0014115762755368799,0.001519935477571203,0.0016221001349896722,0.0017178284445394318,0.0018069266294138426,0.0018892642021272066,0.001964761658147786,0.0020332686615493496,0.002094915038132874,0.0021496133443670348,0.0021977010257414266,0.002238728125864494,0.002273034814167911,0.0023008016011791836,0.0023218538239033575,0.0023362438265207607,0.0023443379335936244,0.002346188149645139,0.002341591816889338,0.0023313495386670284,0.002315241458090272,0.0022936787019615405,0.002266805094889138,0.002235017480737369,0.002198566035895059,0.0021577168462034316,0.0021127691039435808,0.0020640024499177525,0.0020117809676790392,0.001956359880201869,0.0018980510358292558,0.0018370976998262685,0.0017738186591553986,0.0017085294475605677,0.0016415254938129812,0.001573116459746048,0.0015035176917432092,0.0014330790285197704,0.00136204083290026,0.0012907943377894076,0.0012195191539385937,0.0011485042821481096,0.001078061381627579,0.0010083663299140725,9.396483665027597E-4,8.721820584681918E-4,0.005386907564224238" />
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
// IPFS_FILES : LowFIR500.vo
// RELATED_FILES: LowFIR500.v, dspba_library_package.vhd, dspba_library.vhd, auk_dspip_math_pkg_hpfir.vhd, auk_dspip_lib_pkg_hpfir.vhd, auk_dspip_avalon_streaming_controller_hpfir.vhd, auk_dspip_avalon_streaming_sink_hpfir.vhd, auk_dspip_avalon_streaming_source_hpfir.vhd, auk_dspip_roundsat_hpfir.vhd, altera_avalon_sc_fifo.v, LowFIR500_0002_rtl_core.vhd, LowFIR500_0002_ast.vhd, LowFIR500_0002.vhd
