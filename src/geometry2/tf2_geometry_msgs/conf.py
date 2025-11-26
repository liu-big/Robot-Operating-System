# -*- coding: utf-8 -*-
#
# conf.py
#
# Copyright (c) 2024, UCAR.AI, Inc. All rights reserved.
#
# tf2_geometry_msgs 文档构建配置文件，由 sphinx-quickstart 在 2018 年 2 月 13 日创建。
#
# 此文件在执行时，当前目录设置为其包含目录。
#
# 请注意，此自动生成文件中并非包含所有可能的配置值。
#
# 所有配置值都有默认值；被注释掉的值用于显示默认值。

import sys
import os

# 如果扩展（或要用 autodoc 记录的模块）位于另一个目录中，
# 将这些目录添加到 sys.path。如果目录相对于文档根目录，
# 使用 os.path.abspath 使其成为绝对路径，如下所示。
# sys.path.insert(0, os.path.abspath('.'))

# -- 通用配置 ------------------------------------------------

# 如果您的文档需要最小 Sphinx 版本，请在此处声明。
# needs_sphinx = '1.0'

# 在此处添加任何 Sphinx 扩展模块名称，作为字符串。它们可以是
# Sphinx 自带的扩展（命名为 'sphinx.ext.*'）或您的自定义扩展。
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.pngmath',
    'sphinx.ext.viewcode',
]

# 在此处添加任何包含模板的路径，相对于此目录。
templates_path = ['_templates']

# 源文件名的后缀。
# 您可以指定多个后缀作为字符串列表：
# source_suffix = ['.rst', '.md']
source_suffix = '.rst'

# 源文件的编码。
# source_encoding = 'utf-8-sig'

# 主目录树文档。
master_doc = 'index'

# 项目的通用信息。
project = u'tf2_geometry_msgs'
copyright = u'2018, Open Source Robotics Foundation, Inc.'
author = u'Tully Foote'

# 文档项目版本信息，替换 |version| 和 |release|，也用于
# 构建文档中的其他地方。
#
# 短 X.Y 版本。
version = u'0.1'
# 完整版本，包括 alpha/beta/rc 标签。
release = u'0.1'

# Sphinx 自动生成内容的语言。请参阅文档以获取支持的语言列表。
#
# 如果您通过 gettext 目录进行内容翻译，也会使用此项。
# 通常您通过命令行设置“language”。
language = None

# 替换 |today| 有两种选择：
# 1. 如果您将 today 设置为非假值，则使用它：
# today = ''
# 2. 否则，today_fmt 用作 strftime 调用的格式。
# today_fmt = '%B %d, %Y'

# 模式列表，相对于源目录，用于匹配要忽略的文件和目录。
exclude_patterns = ['_build']

# reST 默认角色（用于此标记：`text`），用于所有文档。
# default_role = None

# 如果为 True，则会在 :func: 等交叉引用文本后附加 '()'。
# add_function_parentheses = True

# 如果为 True，则会在所有描述单元标题（如 .. function::）前添加当前模块名称。
# add_module_names = True

# 如果为 True，则会在输出中显示 sectionauthor 和 moduleauthor 指令。默认情况下它们被忽略。
# show_authors = False

# 要使用的 Pygments（语法高亮）样式名称。
pygments_style = 'sphinx'

# 忽略的模块索引排序前缀列表。
# modindex_common_prefix = []

# 如果为 True，则在构建文档中将警告保留为“系统消息”段落。
# keep_warnings = False

# -- HTML 输出选项 ---------------------------------------------------

# 用于 HTML 和 HTML 帮助页面的主题。Sphinx 自带的主要主题有 'default' 和 'sphinxdoc'。
html_theme = 'default'

# 主题选项是主题特定的，并自定义主题的外观和感觉。
# 有关每个主题可用选项的列表，请参阅文档。
# html_theme_options = {}

# 在此处添加任何包含自定义主题的路径，相对于此目录。
# html_theme_path = []

# 此组 Sphinx 文档的名称。如果为 None，则默认为 "<project> v<release> documentation"。
# html_title = None

# 导航栏的较短标题。默认为与 html_title 相同。
# html_short_title = None

# 图像文件（相对于此目录）的名称，放置在侧边栏顶部。
# html_logo = None

# 图像文件（相对于此目录）的名称，用作文档的 favicon。此文件应为 16x16 或 32x32 像素大小的 Windows 图标文件（.ico）。
# html_favicon = None

# 在此处添加任何包含自定义静态文件（如样式表）的路径，相对于此目录。这些文件在内置静态文件之后复制，
# 因此名为 "default.css" 的文件将覆盖内置的 "default.css"。
html_static_path = ['_static']

# 在此处添加任何包含自定义文件（如 robots.txt 或 .htaccess）的额外路径，相对于此目录。这些文件直接复制到文档根目录。
# html_extra_path = []

# 如果不为空，则会在每个页面底部插入“最后更新于:”时间戳，使用给定的 strftime 格式。
# html_last_updated_fmt = '%b %d, %Y'

# 如果为 True，则 SmartyPants 将用于将引号和破折号转换为排版正确的实体。
# html_use_smartypants = True

# 自定义侧边栏模板，将文档名称映射到模板名称。
# html_sidebars = {}

# 应渲染到页面的其他模板，将页面名称映射到模板名称。
# html_additional_pages = {}

# 如果为 False，则不生成模块索引。
# html_domain_indices = True

# 如果为 False，则不生成索引。
# html_use_index = True

# 如果为 True，则索引将拆分为每个字母的单独页面。
# html_split_index = False

# 如果为 True，则会将 reST 源的链接添加到页面。
# html_show_sourcelink = True

# 如果为 True，则会在 HTML 页脚中显示“Created using Sphinx”。默认为 True。
# html_show_sphinx = True

# 如果为 True，则会在 HTML 页脚中显示“(C) Copyright ...”。默认为 True。
# html_show_copyright = True

# 如果为 True，则会输出 OpenSearch 描述文件，并且所有页面都将包含一个引用它的 <link> 标签。此选项的值必须是提供最终 HTML 的基本 URL。
# html_use_opensearch = ''

# 这是 HTML 文件的文件名前缀（例如“.xhtml”）。
# html_file_suffix = None

# 用于生成 HTML 全文搜索索引的语言。
# Sphinx 支持以下语言：
#   'da', 'de', 'en', 'es', 'fi', 'fr', 'hu', 'it', 'ja'
#   'nl', 'no', 'pt', 'ro', 'ru', 'sv', 'tr'
# html_search_language = 'en'

# 包含搜索语言支持选项的字典，默认为空。
# 现在只有 'ja' 使用此配置值
# html_search_options = {'type': 'default'}

# 图像文件（相对于配置目录）的名称，用于实现搜索结果评分器。如果为空，则使用默认值。
# html_search_scorer = 'scorer.js'

# HTML 帮助构建器的输出文件基本名称。
htmlhelp_basename = 'tf2_geometry_msgsdoc'

# -- LaTeX 输出选项 ---------------------------------------------

latex_elements = {
# 纸张大小（'letterpaper' 或 'a4paper'）。
# 'papersize': 'letterpaper',

# 字体大小（'10pt'、'11pt' 或 '12pt'）。
# 'pointsize': '10pt',

# LaTeX 序言的附加内容。
# 'preamble': '',

# LaTeX 浮动（float）对齐。
# 'figure_align': 'htbp',
}

# 将文档树分组到 LaTeX 文件中。元组列表
# （源起始文件、目标名称、标题、
#  作者、文档类 [howto、manual 或自定义类]）。
latex_documents = [
    (master_doc, 'tf2_geometry_msgs.tex', u'tf2\\_geometry\\_msgs Documentation',
     u'Tully Foote', 'manual'),
]

# 图像文件（相对于此目录）的名称，放置在标题页顶部。
# latex_logo = None

# 对于“manual”文档，如果为 True，则顶级标题为部分，而不是章节。
# latex_use_parts = False

# 如果为 True，则在内部链接后显示页面引用。
# latex_show_pagerefs = False

# 如果为 True，则在外部链接后显示 URL 地址。
# latex_show_urls = False

# 附加到所有手册的文档。
# latex_appendices = []

# 如果为 False，则不生成模块索引。
# latex_domain_indices = True


# -- 手册页输出选项 ---------------------------------------

# 每个手册页一个条目。元组列表
# （源起始文件、名称、描述、作者、手册部分）。
man_pages = [
    (master_doc, 'tf2_geometry_msgs', u'tf2_geometry_msgs Documentation',
     [author], 1)
]

# 如果为 True，则在外部链接后显示 URL 地址。
# man_show_urls = False


# -- Texinfo 输出选项 -------------------------------------------

# 将文档树分组到 Texinfo 文件中。元组列表
# （源起始文件、目标名称、标题、作者、
#  目录菜单条目、描述、类别）。
texinfo_documents = [
    (master_doc, 'tf2_geometry_msgs', u'tf2_geometry_msgs Documentation',
     author, 'tf2_geometry_msgs', 'One line description of project.',
     'Miscellaneous'),
]

# 附加到所有手册的文档。
# texinfo_appendices = []

# 如果为 False，则不生成模块索引。
# texinfo_domain_indices = True

# 如何显示 URL 地址：'footnote'、'no' 或 'inline'。
# texinfo_show_urls = 'footnote'

# 如果为 True，则不在“Top”节点的菜单中生成 @detailmenu。
# texinfo_no_detailmenu = False


# intersphinx 的示例配置：引用 Python 标准库。
intersphinx_mapping = {'https://docs.python.org/': None}
