//
// Created by jun on 7/28/17.
//
#include <iostream>

#include "ego_transition_state.h"
#include "ego_transition_CS_to_FT.h"
#include "ego_transition_CS_to_CLR.h"
#include "ego_transition_CS_to_CLL.h"
#include "ego_transition_CL_to_FT.h"
#include "ego_transition_FT_to_CS.h"
#include "ego_transition_FT_to_CLR.h"
#include "ego_transition_FT_to_CLL.h"

/*
 * EgoTransitionState class
 */
EgoTransitionState::EgoTransitionState() {}

EgoTransitionState::~EgoTransitionState() {}


/*
 * EgoTransitionStateFactory class
 */

EgoTransitionStateFactory::EgoTransitionStateFactory() {}

EgoTransitionStateFactory::~EgoTransitionStateFactory() {}

EgoTransitionState* EgoTransitionStateFactory::createState(TransitionStates name) {
  switch (name) {
    case CS_TO_FT:
      static EgoTransitionCSToFT cs_to_ft;
      return &cs_to_ft;
    case CS_TO_CLR:
      static EgoTransitionCSToCLR cs_to_clr;
      return &cs_to_clr;
    case CS_TO_CLL:
      static EgoTransitionCSToCLL cs_to_cll;
      return &cs_to_cll;
    case CL_TO_FT:
      static EgoTransitionCLToFT cl_to_ft;
      return &cl_to_ft;
    case FT_TO_CLR:
      static EgoTransitionFTToCLR ft_to_clr;
      return &ft_to_clr;
    case FT_TO_CLL:
      static EgoTransitionFTToCLL ft_to_cll;
      return &ft_to_cll;
    case FT_TO_CS:
      static EgoTransitionFTToCS ft_to_cs;
      return &ft_to_cs;
    default:
      throw std::invalid_argument("Unknown transition state!");
  }
}
