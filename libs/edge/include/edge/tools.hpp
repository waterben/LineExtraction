Edge Response Filter(ERF)
    : -Gradient - Laplace - Quadrature -
      PhaseCongruency

      Edge Tools : -EdgePixelExtractors(EPE)
    : -NonMaximumSuppression(NMS) - ZeroCrossing(ZC) - (Sub)PixelEstimator(SPE) - EdgeSource<ERD, EPE>(ES) -
      LineTracer(LT) - NFA(NFA) - LineFit(LF) - LineSplit(LS) -
      EdgeSegmentDetectors(ESD) - EsdDrawing - EsdLinking - EsdSimple -
      EsdPattern

      Line Segment Detector classes
    : -LdHough<ES> -
      LsdHough<LT, SPE, LF, ES> - LsdFGioi - LsdEDLZ - LsdHoughP<ES> - LsdBurns<ERF, LF> - LsdFBW<ERV, LF> -
      LsdCC<ERF, LF> - LsdCP<ERF, LF> - LsdEL<ES, ESD, NFA, SPE, LS, LF> -
      LsdEP<ES, SPE, LS, LF>

      Inheritance : ValueManager <
    -EdgeSourceI < -EdgeSourceT < -EdgeSourceNMS_ZC... ValueManager < -EsdBase < -EsdDrawing < -EsdLinking <
    -EsdSimple < -EsdBasePattern < -EsdPattern ValueManager < -LdBase < -LdHough < -LsdBase < -LsdHough < -LsdFGioi <
    -LsdEDLZ < -LsdHoughP < -LsdBurns < -LsdFBW < -LsdCCBase < -LsdCC < -LsdCP < -LsdExt < -LsdEL < -LsdEP
