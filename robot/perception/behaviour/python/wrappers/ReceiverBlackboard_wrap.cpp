class_<ReceiverBlackboard>("ReceiverBlackboard")
	.def_readonly("incapacitated"  , &ReceiverBlackboard::incapacitated)
	.def_readonly("lastReceived"   , &ReceiverBlackboard::lastReceived )
	.def_readonly("message"        , &ReceiverBlackboard::message      )
	.def_readonly("data"           , &ReceiverBlackboard::data         );
