function y = norm_pdf(x, mu, sigma)

if sigma <= 0
    sigma = eps;
end

y = (1/(sigma*sqrt(2*pi))) * exp(-((x-mu).^2)/(2*sigma^2));

end