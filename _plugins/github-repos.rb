# Fetches GitHub user and repository metadata at build time so the repositories
# page can render its own cards instead of hotlinking github-readme-stats.
#
# Populates:
#   site.data.github_profiles   {login => user hash}
#   site.data.github_repo_cards {"owner/name" => repo hash}
#
# Both are keyed by exactly the strings in _data/repositories.yml, so the
# includes can look a card up directly rather than searching.
#
# Responses are cached under .jekyll-cache/github/ (gitignored) so repeated
# local builds don't re-hit the API. Set GITHUB_TOKEN to lift the anonymous
# 60-requests/hour limit to 5000 — the Actions workflow passes it in.
#
# Every failure path is soft: a request that fails falls back to stale cache,
# and failing that leaves the card absent. The includes render a plain link in
# that case. A GitHub outage must never break the build.

require 'json'
require 'net/http'
require 'uri'
require 'fileutils'
require 'jekyll'

module GithubRepos
  class Generator < Jekyll::Generator
    safe true
    priority :high

    DEFAULT_TTL = 3600 # seconds

    def generate(site)
      data = site.data['repositories']
      return if data.nil?

      @cache_dir = File.join(site.source, '.jekyll-cache', 'github')
      FileUtils.mkdir_p(@cache_dir)
      ttl = (site.config['github_api_cache_ttl'] || DEFAULT_TTL).to_i

      profiles = {}
      Array(data['github_users']).each do |login|
        profile = fetch("users/#{login}", ttl)
        next if profile.nil?

        # Total stars is not on the user object; sum it from their own repos.
        # Must paginate — this account has more repos than one page holds, and
        # a single page would silently undercount.
        owned = fetch_owned_repos(login, ttl)
        profile['total_stars'] =
          owned.nil? ? nil : owned.sum { |r| r['stargazers_count'].to_i }
        profiles[login] = profile
      end

      cards = {}
      Array(data['github_repos']).each do |slug|
        card = fetch("repos/#{slug}", ttl)
        cards[slug] = card unless card.nil?
      end

      site.data['github_profiles'] = profiles
      site.data['github_repo_cards'] = cards

      missing = Array(data['github_repos']).size - cards.size
      if missing > 0
        Jekyll.logger.warn 'GitHub:', "#{missing} repo card(s) unavailable; rendering plain links"
      end
    end

    private

    PER_PAGE = 100
    MAX_PAGES = 10 # backstop; 1000 repos is far beyond any real account here

    # Walks every page of a user's owned repos. Returns nil (rather than a
    # partial list) if any page fails, so a half-fetched total is never
    # presented as a real number.
    def fetch_owned_repos(login, ttl)
      all = []
      (1..MAX_PAGES).each do |page|
        chunk = fetch("users/#{login}/repos?per_page=#{PER_PAGE}&type=owner&page=#{page}", ttl)
        return nil unless chunk.is_a?(Array)

        all.concat(chunk)
        break if chunk.size < PER_PAGE
      end
      all
    end

    def fetch(path, ttl)
      file = File.join(@cache_dir, "#{path.gsub(/[^a-zA-Z0-9]/, '_')}.json")

      if File.exist?(file) && (Time.now - File.mtime(file)) < ttl
        return JSON.parse(File.read(file))
      end

      res = request(path)
      if res.is_a?(Net::HTTPSuccess)
        body = JSON.parse(res.body)
        File.write(file, JSON.generate(body))
        body
      else
        Jekyll.logger.warn 'GitHub:', "#{path} -> HTTP #{res.code}"
        stale(file)
      end
    rescue StandardError => e
      Jekyll.logger.warn 'GitHub:', "#{path} failed (#{e.class}: #{e.message})"
      stale(file)
    end

    def request(path)
      uri = URI("https://api.github.com/#{path}")
      req = Net::HTTP::Get.new(uri)
      req['Accept'] = 'application/vnd.github+json'
      req['User-Agent'] = 'jekyll-github-repos'
      token = ENV['GITHUB_TOKEN'] || ENV['GH_TOKEN']
      req['Authorization'] = "Bearer #{token}" unless token.nil? || token.empty?

      Net::HTTP.start(uri.hostname, uri.port, use_ssl: true,
                      open_timeout: 10, read_timeout: 15) do |http|
        http.request(req)
      end
    end

    # Expired cache still beats no card at all when the API is unreachable.
    def stale(file)
      return nil unless file && File.exist?(file)

      Jekyll.logger.info 'GitHub:', "using stale cache for #{File.basename(file)}"
      JSON.parse(File.read(file))
    end
  end
end
