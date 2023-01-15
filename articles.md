<!--

---
layout: default
title: Articles
---
# Articles

{% for item in site.data.articlesList %}
<h2>{{ item.title }}</h2>
  <ul>
    {% for entry in item.subfolderitems %}
      <li>
        <h3><a href="{{ entry.url }}">{{ entry.name }}</a></h3>
        {% assign article_page = site.pages | where: "path", entry.path | first %}
        <p>{{ article_page.excerpt }}</p>
        <a href="{{ entry.url }}" style="color: blue; color: blue; text-decoration: none;"> ... continue reading</a>
      </li>
    {% endfor %}
  </ul>
{% endfor %}

-->