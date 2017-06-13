%UT_MWEIGHTS - Generate matrix form unscented transformation weights
%
% Syntax:
%   [WM,W,c] = ut_mweights(n,alpha,beta,kappa)
%
% In:
%   n     - Dimensionality of random variable
%   alpha - Transformation parameter  (optional, default 0.5)
%   beta  - Transformation parameter  (optional, default 2)
%   kappa - Transformation parameter  (optional, default 3-size(X,1))
%
% Out:
%   WM - Weight vector for mean calculation
%    W - Weight matrix for covariance calculation
%    c - Scaling constant
%
% Description:
%   Computes matrix form unscented transformation weights.
%

% Copyright (C) 2006 Simo S?rkk?
%
% $Id: ut_mweights.m 109 2007-09-04 08:32:58Z jmjharti $
%
% This software is distributed under the GNU General Public 
% Licence (version 2 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [WM,W,c, WC] = ut_mweights(n,alpha,beta,kappa)


  [WM,WC,c] = ut_weights(n,alpha,beta,kappa);

  W = eye(length(WC)) - repmat(WM,1,length(WM));
  W = W * diag(WC) * W';
