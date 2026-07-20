%hdl gen script
cfg = coder.config('fixpt');
cfg.TestBenchName = 'golden_cov';
cfg.DefaultWordLength = 16;
cfg.ProposeFractionLengthsForDefaultWordLength = true;
cfg.fimath = fimath('RoundingMethod','Nearest','OverflowAction','Saturate', ...
    'ProductMode','SpecifyPrecision','ProductWordLength',40,'ProductFractionLength',30, ...
    'SumMode','SpecifyPrecision','SumWordLength',40,'SumFractionLength',30);
cfg.addTypeSpecification('covariance_core', 'R', numerictype(1,40,30));
fprintf('cfg class: %s (expect coder.FixPtConfig)\n', class(cfg));

hdlcfg = coder.config('hdl');
hdlcfg.TestBenchName = 'golden_cov';
hdlcfg.TargetLanguage = 'Verilog';
hdlcfg.GenerateHDLTestBench = true;
hdlcfg.LoopOptimization = 'StreamLoops';
fprintf('hdlcfg class: %s (expect coder.HdlConfig)\n', class(hdlcfg));

codegen('-float2fixed', cfg, '-config', hdlcfg, 'covariance_core');