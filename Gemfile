source "https://rubygems.org"

# Building via GitHub Actions (see .github/workflows/jekyll.yml) instead of
# GitHub's classic Pages build pipeline, whose frozen "github-pages" gem
# could no longer resolve this Gemfile's dependencies. Actions installs
# whatever's listed here fresh, so we depend on Jekyll and each plugin
# directly instead of through that meta-gem.
gem "jekyll", "~> 4.3"

gem "tzinfo-data"
gem "wdm", "~> 0.1.0" if Gem.win_platform?

# If you have any plugins, put them here!
group :jekyll_plugins do
  gem "jekyll-remote-theme"
  gem "jekyll-paginate"
  gem "jekyll-sitemap"
  gem "jekyll-gist"
  gem "jekyll-feed"
  gem "jemoji"
  gem "jekyll-include-cache"
end
