---
layout: page
title: Papers
permalink: /papers/
---

# Papers I've Read

This page contains summaries and notes for the papers I have read. I update this list as I read new papers.

{% if site.papers and site.papers != empty %}
{% for paper in site.papers %}
### [{{ paper.title }}]({{ paper.url }})
{% if paper.authors %}**Authors:** {{ paper.authors }}

{% endif %}
{{ paper.excerpt | markdownify }}

---
{% endfor %}
{% else %}
No papers yet — add markdown files to the `_papers` folder.
{% endif %}
